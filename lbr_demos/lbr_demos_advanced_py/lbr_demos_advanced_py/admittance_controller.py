import numpy as np
import optas

from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState


class AdmittanceController(object):
    def __init__(
        self,
        robot_description: str,
        base_link: str = "lbr_link_0",
        end_effector_link: str = "lbr_link_ee",
        f_ext_th: np.ndarray = np.array([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]),
        dq_gains: np.ndarray = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        dx_gains: np.ndarray = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
    ) -> None:
        self._lbr_joint_position_command = LBRJointPositionCommand()

        self._robot = optas.RobotModel(urdf_string=robot_description)

        self._jacobian_func = self._robot.get_link_geometric_jacobian_function(
            link=end_effector_link, base_link=base_link, numpy_output=True
        )

        self._dof = self._robot.ndof
        self._jacobian = np.zeros((6, self._dof))
        self._jacobian_inv = np.zeros((self._dof, 6))
        self._q = np.zeros(self._dof)
        self._dq = np.zeros(self._dof)
        self._tau_ext = np.zeros(6)
        self._dq_gains = np.diag(dq_gains)
        self._dx_gains = np.diag(dx_gains)
        self._f_ext = np.zeros(6)
        self._f_ext_th = f_ext_th
        self._alpha = 0.95

    def __call__(self, lbr_state: LBRState, dt: float) -> LBRJointPositionCommand:
        self._q = np.array(lbr_state.measured_joint_position.tolist())
        self._tau_ext = np.array(lbr_state.external_torque.tolist())

        self._jacobian = self._jacobian_func(self._q)
        self._jacobian_inv = np.linalg.pinv(self._jacobian, rcond=0.1)
        self._f_ext = self._jacobian_inv.T @ self._tau_ext

        dx = np.where(
            abs(self._f_ext) > self._f_ext_th,
            self._dx_gains @ np.sign(self._f_ext) * (abs(self._f_ext) - self._f_ext_th),
            0.0,
        )

        # additional smoothing required in python
        self._dq = (
            self._alpha * self._dq
            + (1 - self._alpha) * self._dq_gains @ self._jacobian_inv @ dx
        )

        self._lbr_joint_position_command.joint_position = (
            np.array(lbr_state.measured_joint_position.tolist()) + dt * self._dq
        ).data

        return self._lbr_joint_position_command
