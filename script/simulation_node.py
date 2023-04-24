#!/usr/bin/env python
import rospy
from simulator import Simulator


def main():
    rospy.init_node('simulation_node', anonymous=True)
    rospy.loginfo("Start simulation node")

    sim = Simulator()
    rospy.spin()

if __name__ == '__main__':
    main()
