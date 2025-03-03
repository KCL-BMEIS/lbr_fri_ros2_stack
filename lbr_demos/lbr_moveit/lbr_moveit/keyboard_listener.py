import numpy as np
from pynput.keyboard import Key, Listener

from .forward_keyboard_node import ForwardKeyboardNode


class KeyboardListener:
    def __init__(self, command_forward_node: ForwardKeyboardNode):
        self._command_forward_node = command_forward_node
        self._key_listener = Listener(
            on_press=self._on_key_press, on_release=self._on_key_release
        )
        self._valid_numbers = None
        self._twist_cmd = np.zeros(6)
        self._joint_velocity_cmd = None
        self._joint_vel_direction = 1.0

    def __enter__(self):
        self._key_listener.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self._command_forward_node.get_logger().info("Exiting keyboard listener with.")
        self._key_listener.stop()

    def _on_key_release(self, key: Key) -> None:
        key_str = str(key).replace("'", "")
        # zero out the velocity commands
        try:
            if (
                key_str
                == self._command_forward_node.keyboard_layout.translation.z.increase
            ):
                self._twist_cmd[2] = 0.0  # translation along z-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.z.decrease
            ):
                self._twist_cmd[2] = 0.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.z.increase
            ):
                self._twist_cmd[5] = 0.0  # rotation about z-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.z.decrease
            ):
                self._twist_cmd[5] = 0.0
            elif key_str == self._command_forward_node.keyboard_layout.pause:  # pause
                self._twist_cmd = np.zeros(6)
                self._joint_velocity_cmd = np.zeros(self._command_forward_node.dof)
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.x.increase
            ):
                self._twist_cmd[0] = 0.0  # translation along x-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.x.decrease
            ):
                self._twist_cmd[0] = 0.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.y.increase
            ):
                self._twist_cmd[1] = 0.0  # translation along y-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.y.decrease
            ):
                self._twist_cmd[1] = 0.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.x.increase
            ):
                self._twist_cmd[3] = 0.0  # rotation about x-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.x.decrease
            ):
                self._twist_cmd[3] = 0.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.y.increase
            ):
                self._twist_cmd[4] = 0.0  # rotation about y-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.y.decrease
            ):
                self._twist_cmd[4] = 0.0
            elif key_str in self._valid_numbers:
                # get index of key_str within valid_numbers
                index = self._valid_numbers.index(key_str)
                if 0 <= index < self._command_forward_node.dof:
                    self._joint_velocity_cmd[index] = 0.0
            self._command_forward_node.twist_cmd = self._twist_cmd
            self._command_forward_node.joint_veloctiy_cmd = self._joint_velocity_cmd
        except AttributeError:
            pass

    def _on_key_press(self, key: Key) -> None:
        key_str = str(key).replace("'", "")
        if key_str == self._command_forward_node.keyboard_layout.escape:
            self._command_forward_node.get_logger().info("Exiting keyboard listener.")
            self._twist_cmd = np.zeros(6)
            self._joint_velocity_cmd = np.zeros(self._command_forward_node.dof)
            self._command_forward_node.twist_cmd = self._twist_cmd
            self._command_forward_node.joint_veloctiy_cmd = self._joint_velocity_cmd
            self._key_listener.stop()
        if not self._command_forward_node.dof:
            return
        if not self._valid_numbers:
            self._valid_numbers = "".join(
                [str(i) for i in self._command_forward_node.keyboard_layout.joints]
            )
            self._joint_velocity_cmd = np.zeros(self._command_forward_node.dof)
        try:
            if (
                key_str
                == self._command_forward_node.keyboard_layout.translation.z.increase
            ):
                self._twist_cmd[2] = 1.0  # translation along z-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.z.decrease
            ):
                self._twist_cmd[2] = -1.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.z.increase
            ):
                self._twist_cmd[5] = 1.0  # rotation about z-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.z.decrease
            ):
                self._twist_cmd[5] = -1.0
            elif key_str == self._command_forward_node.keyboard_layout.pause:  # pause
                self._twist_cmd = np.zeros(6)
                self._joint_velocity_cmd = np.zeros(self._command_forward_node.dof)
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.x.increase
            ):
                self._twist_cmd[0] = 1.0  # translation along x-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.x.decrease
            ):
                self._twist_cmd[0] = -1.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.y.increase
            ):
                self._twist_cmd[1] = 1.0  # translation along y-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.translation.y.decrease
            ):
                self._twist_cmd[1] = -1.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.x.increase
            ):
                self._twist_cmd[3] = 1.0  # rotation about x-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.x.decrease
            ):
                self._twist_cmd[3] = -1.0
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.y.increase
            ):
                self._twist_cmd[4] = 1.0  # rotation about y-axis
            elif (
                key_str
                == self._command_forward_node.keyboard_layout.rotation.y.decrease
            ):
                self._twist_cmd[4] = -1.0
            elif key_str == self._command_forward_node.keyboard_layout.reverse_joints:
                self._joint_vel_direction *= -1.0
            elif key_str in self._valid_numbers:
                # get index of key_str within valid_numbers
                index = self._valid_numbers.index(key_str)
                if 0 <= index < self._command_forward_node.dof:
                    self._joint_velocity_cmd[index] = self._joint_vel_direction
            self._command_forward_node.twist_cmd = self._twist_cmd
            self._command_forward_node.joint_veloctiy_cmd = self._joint_velocity_cmd
        except AttributeError:
            pass
