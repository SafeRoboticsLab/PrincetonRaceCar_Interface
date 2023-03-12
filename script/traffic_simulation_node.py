#!/usr/bin/env python
import rospy
from simulator import TrafficSimulator


def main():
    rospy.init_node('traffic_simulation_node')
    rospy.loginfo("Start traffic simulation node")
    map_file = rospy.get_param("~map_file")
    sim = TrafficSimulator(map_file)
    rospy.spin()

if __name__ == '__main__':
    main()
