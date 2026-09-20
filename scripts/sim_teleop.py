"""
Minimal keyboard teleop for the euroc_mav_sim rig -- publishes geometry_msgs/Twist
on /cmd_vel, which the sim's VelocityControl plugin (see sim/worlds/euroc_mav_sim.sdf)
applies directly to the rig's body. Unlike stock teleop_twist_keyboard, this includes
z-axis (altitude) control, since the rig is a MAV and can actually move vertically.

Run alongside the sim + bridge (ros2 launch sensor_pkg sim.launch.py):
    python3 scripts/sim_teleop.py

Keys (hold Shift is NOT needed, just tap):
    i / ,   forward / backward   (linear.x)
    j / l   yaw left / right     (angular.z)
    u / o   ascend / descend     (linear.z)
    k       stop (zero everything)
    Ctrl-C  quit (publishes zero twist first)
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.5   # m/s
VERTICAL_SPEED = 0.3 # m/s
ANGULAR_SPEED = 0.5  # rad/s

BINDINGS = {
    'i': (LINEAR_SPEED, 0.0, 0.0),
    ',': (-LINEAR_SPEED, 0.0, 0.0),
    'j': (0.0, 0.0, ANGULAR_SPEED),
    'l': (0.0, 0.0, -ANGULAR_SPEED),
    'u': (0.0, VERTICAL_SPEED, 0.0),
    'o': (0.0, -VERTICAL_SPEED, 0.0),
    'k': (0.0, 0.0, 0.0),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = Node('sim_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    settings = termios.tcgetattr(sys.stdin.fileno())

    print(__doc__)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':  # Ctrl-C
                break
            if key not in BINDINGS:
                continue

            vx, vz, wz = BINDINGS[key]
            twist = Twist()
            twist.linear.x = vx
            twist.linear.z = vz
            twist.angular.z = wz
            pub.publish(twist)
    finally:
        pub.publish(Twist())  # zero twist on exit so the rig doesn't keep coasting
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
