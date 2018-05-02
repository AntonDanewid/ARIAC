#!/usr/bin/env python

from ariac_example import ariac_planner
import rospy


def main():
    rospy.init_node("planner_node")
    planner = ariac_planner.Planner()
    planner.__init__()
    rospy.loginfo("Setup complete.")
    ariac_planner.start_competition(planner)
    rospy.spin()



if __name__ == '__main__':
    main()
