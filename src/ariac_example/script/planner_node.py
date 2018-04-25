#!/usr/bin/env python

from ariac_example import ariac_planner
import rospy


def main():
    rospy.init_node("planner_node")

    planner = ariac_planner.Planner()

    rospy.loginfo("Setup complete.")
    ariac_planner.start_competition()




if __name__ == '__main__':
    main()
