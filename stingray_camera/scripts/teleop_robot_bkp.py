#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Read a single keypress from stdin (blocking)"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rospy.init_node('teleop_terminal')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    LIN_SPEED = 0.2
    ANG_SPEED = 1.0

    print("Use W/A/S/D to move, Q/E to rotate. X to stop. ESC to quit.")

    while not rospy.is_shutdown():
        key = get_key()

        twist = Twist()
        if key == 'w':
            twist.linear.x = LIN_SPEED
        elif key == 's':
            twist.linear.x = -LIN_SPEED
        elif key == 'a':
            twist.linear.y = LIN_SPEED
        elif key == 'd':
            twist.linear.y = -LIN_SPEED
        elif key == 'q':
            twist.angular.z = ANG_SPEED
        elif key == 'e':
            twist.angular.z = -ANG_SPEED
        elif key == 'x':
            twist = Twist()  # stop
        elif ord(key) == 27:  # ESC
            print("\nExiting...")
            break

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

