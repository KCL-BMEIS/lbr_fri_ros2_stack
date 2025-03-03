import rclpy
from lbr_moveit.forward_keyboard_node import ForwardKeyboardNode
from lbr_moveit.keyboard_listener import KeyboardListener


def main():
    rclpy.init()
    forward_keyboard_node = ForwardKeyboardNode()
    try:
        with KeyboardListener(forward_keyboard_node):
            rclpy.spin(forward_keyboard_node)
    except KeyboardInterrupt:
        pass
