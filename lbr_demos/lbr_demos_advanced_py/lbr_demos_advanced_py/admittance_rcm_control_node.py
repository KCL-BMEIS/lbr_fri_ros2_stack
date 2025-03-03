import numpy as np
import rclpy

from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState

from .admittance_rcm_controller import AdmittanceRCMController
from .lbr_base_position_command_node import LBRBasePositionCommandNode


class LBRAdmittanceControlRCMNode(LBRBasePositionCommandNode):
    def __init__(self, node_name: str = "admittance_rcm_control") -> None:
        super().__init__(node_name)

        # declare and get parameters
        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("end_effector_link", "lbr_link_ee")
        self.declare_parameter("f_ext_th", [4.0, 4.0, 4.0, 1.0, 1.0, 1.0])
        self.declare_parameter("dq_gains", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("dx_gains", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.declare_parameter("exp_smooth", 0.95)

        self._f_ext_th = np.array(
            self.get_parameter("f_ext_th").get_parameter_value().double_array_value
        )
        self._dq_gains = np.diag(
            self.get_parameter("dq_gains").get_parameter_value().double_array_value
        )
        self._dx_gains = np.diag(
            self.get_parameter("dx_gains").get_parameter_value().double_array_value
        )

        self._exp_smooth = (
            self.get_parameter("exp_smooth").get_parameter_value().double_value
        )
        if self._exp_smooth < 0.0 or self._exp_smooth > 1.0:
            raise ValueError("Exponential smoothing factor must be in [0, 1].")

        self._dq = np.zeros(7)
        self._controller = AdmittanceRCMController(
            robot_description=self._robot_description,
            base_link=self.get_parameter("base_link")
            .get_parameter_value()
            .string_value,
            end_effector_link=self.get_parameter("end_effector_link")
            .get_parameter_value()
            .string_value,
        )

        # log parameters to terminal
        self._log_parameters()

    def _log_parameters(self) -> None:
        self.get_logger().info("*** Paramters:")
        self.get_logger().info(
            f"*   base_link: {self.get_parameter('base_link').value}"
        )
        self.get_logger().info(
            f"*   end_effector_link: {self.get_parameter('end_effector_link').value}"
        )
        self.get_logger().info(f"*   f_ext_th: {self.get_parameter('f_ext_th').value}")
        self.get_logger().info(f"*   dq_gains: {self.get_parameter('dq_gains').value}")
        self.get_logger().info(f"*   dx_gains: {self.get_parameter('dx_gains').value}")
        self.get_logger().info(
            f"*   exp_smooth: {self.get_parameter('exp_smooth').value}"
        )

    def _command(self, q) -> None:
        lbr_command = LBRJointPositionCommand()
        lbr_command.joint_position = q
        self._lbr_joint_position_command_pub.publish(lbr_command)

    def _admittance(self, tau_ext, qc) -> None:
        J = self._controller.jacobian_func(qc)
        Jinv = np.linalg.pinv(J, rcond=0.1)
        f_ext = Jinv.T @ tau_ext
        dx = np.where(
            abs(f_ext) > self._f_ext_th,
            self._dx_gains @ np.sign(f_ext) * (abs(f_ext) - self._f_ext_th),
            0.0,
        ).flatten()
        dx = np.clip(dx, -1.0, 1.0)
        dq = Jinv @ dx
        self._dq = self._exp_smooth * self._dq + (1.0 - self._exp_smooth) * dq
        return self._dq

    def _on_lbr_state(self, msg: LBRState) -> None:
        if self._controller._rcm is None:
            self._controller.set_start(msg.measured_joint_position)
            return

        dq_goal = self._admittance(msg.external_torque, msg.measured_joint_position)
        self._controller.reset(msg.measured_joint_position, dq_goal)

        if self._controller.solve():
            qc = np.array(msg.measured_joint_position)
            dq = self._controller.get_qd_target()
            qn = qc + self._dt * self._dq_gains @ dq
            self._command(qn.tolist())
        else:
            self.get_logger().error("Solver failed!")
            qn = np.array(msg.measured_joint_position)
            self._command(qn.tolist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LBRAdmittanceControlRCMNode()
    rclpy.spin(node)
    rclpy.shutdown()
