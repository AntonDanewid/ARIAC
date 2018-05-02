#!/usr/bin/env python

from ariac_example import ariac_planner
import rospy


def main():
    rospy.init_node("planner_node")

    planner = ariac_planner.Planner()

    rospy.loginfo("Setup complete.")
    ariac_planner.connect_callbacks(planner)

if __name__ == '__main__':
    main()
