#!/usr/bin/env python

import rospy, sys
import actionlib
import  pr2_controllers_msgs.msg as pr2c 



if __name__ == "__main__":
        rospy.init_node('open_gripper')
        # Prepare Action Controller for gripper
        ac = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',pr2c.Pr2GripperCommandAction)
        ac.wait_for_server()

        g_open = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.0899, 100))
        ac.send_goal(g_open)
        ac.wait_for_result()
        rospy.spin()
