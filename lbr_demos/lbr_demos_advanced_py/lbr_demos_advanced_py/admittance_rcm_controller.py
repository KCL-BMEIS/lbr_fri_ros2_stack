import casadi as cs
import optas


class AdmittanceRCMController:
    def __init__(
        self,
        robot_description: str,
        base_link: str = "lbr_link_0",
        end_effector_link: str = "lbr_link_ee",
    ):
        self._robot = optas.RobotModel(
            urdf_string=robot_description, time_derivs=[0, 1]
        )

        self.jacobian_func = self._robot.get_link_geometric_jacobian_function(
            link=end_effector_link, base_link=base_link, numpy_output=True
        )

        T = 2
        builder = optas.OptimizationBuilder(T, robots=[self._robot])
        self._name = self._robot.get_name()

        rcm = builder.add_parameter("rcm", 3)

        q0 = builder.get_model_state(self._name, 0, time_deriv=0)
        qF = builder.get_model_state(self._name, 1, time_deriv=0)
        qd = builder.get_model_state(self._name, 0, time_deriv=1)

        qc = builder.add_parameter("qc", self._robot.ndof)
        qd_goal = builder.add_parameter("qd_goal", self._robot.ndof)

        _q = optas.SX.sym("q", self._robot.ndof)
        _rcm = optas.SX.sym("rcm", 3)
        Tf = self._robot.get_global_link_transform(end_effector_link, _q)
        zf = Tf[:3, 2]
        pf = Tf[:3, 3]
        alpha = zf.T @ (_rcm - pf)
        _dist_sqr = optas.sumsqr(pf + alpha * zf - _rcm)
        self._dist_sqr = optas.Function("dist_sqr", [_q, _rcm], [_dist_sqr])

        builder.add_equality_constraint("rcm", self._dist_sqr(qF, rcm))

        c1 = cs.sumsqr(qd - qd_goal)
        builder.add_cost_term("match_qd_goal", c1)

        builder.add_equality_constraint("qinit", q0, qc)
        builder.add_equality_constraint("dynamics", q0 + qd, qF)

        self._eff_transform = self._robot.get_global_link_transform_function(
            end_effector_link
        )

        q = cs.SX.sym("q", self._robot.ndof)
        p = self._robot.get_global_link_position(end_effector_link, q)
        self._xpos = cs.Function("xpos", [q], [p[0]])

        self._solver = optas.ScipyMinimizeSolver(builder.build()).setup("SLSQP")

        self._rcm = None

    def set_start(self, q):
        T = self._eff_transform(q)
        z = T[:3, 2]
        p = T[:3, 3]
        self._rcm = p + 0.2 * z

    def reset(self, qc, qd_goal):
        params = {"qc": qc, "qd_goal": qd_goal, "rcm": self._rcm}
        self._solver.reset_parameters(params)
        self._solver.reset_initial_seed(
            {
                f"{self._name}/q": cs.horzcat(cs.DM(qc), cs.DM(qc)),
                f"{self._name}/dq": qd_goal,
            }
        )

    def solve(self):
        self.sol = self._solver.solve()
        return self._solver.did_solve()

    def get_qd_target(self):
        return self.sol[f"{self._name}/dq"].toarray().flatten()
