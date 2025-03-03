import numpy as np
import rclpy

from lbr_fri_idl.msg import LBRState

from .admittance_controller import AdmittanceController
from .lbr_base_position_command_node import LBRBasePositionCommandNode


class AdmittanceControlNode(LBRBasePositionCommandNode):
    def __init__(self, node_name: str = "admittance_control") -> None:
        super().__init__(node_name=node_name)

        # parameters
        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("end_effector_link", "lbr_link_ee")
        self.declare_parameter("f_ext_th", [2.0, 2.0, 2.0, 0.5, 0.5, 0.5])
        self.declare_parameter("dq_gains", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("dx_gains", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.declare_parameter("exp_smooth", 0.95)

        self._init = False
        self._lbr_state = LBRState()
        self._exp_smooth = (
            self.get_parameter("exp_smooth").get_parameter_value().double_value
        )
        if self._exp_smooth < 0.0 or self._exp_smooth > 1.0:
            raise ValueError("Exponential smoothing factor must be in [0, 1].")

        self._controller = AdmittanceController(
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

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        self._smooth_lbr_state(lbr_state)

        lbr_command = self._controller(self._lbr_state, self._dt)
        self._lbr_joint_position_command_pub.publish(lbr_command)

    def _smooth_lbr_state(self, lbr_state: LBRState) -> None:
        if not self._init:
            self._lbr_state = lbr_state
            self._init = True
            return

        self._lbr_state.measured_joint_position = (
            (1 - self._exp_smooth)
            * np.array(self._lbr_state.measured_joint_position.tolist())
            + self._exp_smooth * np.array(lbr_state.measured_joint_position.tolist())
        ).data

        self._lbr_state.external_torque = (
            (1 - self._exp_smooth) * np.array(self._lbr_state.external_torque.tolist())
            + self._exp_smooth * np.array(lbr_state.external_torque.tolist())
        ).data


def main(args=None) -> None:
    rclpy.init(args=args)
    admittance_control_node = AdmittanceControlNode()
    rclpy.spin(admittance_control_node)
    rclpy.shutdown()
