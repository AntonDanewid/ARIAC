
from ariac_example import ariac_planner
import rospy


def main():
    rospy.loginfo("RUNNING THIS")
    rospy.init_node("ariac_planner_node")

    planner = ariac_planner.MyCompetitionClass()
    ariac_planner.connect_callbacks(comp_class)

    rospy.loginfo("Setup complete.")
    ariac_planner.start_competition()

    if not planner.has_been_zeroed:
        planner.has_been_zeroed = True
        rospy.loginfo("Sending arm to zero joint positions... TESTIN")
        planner.send_arm_to_state([0] * len(comp_class.arm_joint_names))

    rospy.spin()


if __name__ == '__main__':
    main()
