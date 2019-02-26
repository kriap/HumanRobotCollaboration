#!/usr/bin/env python

import sys
import rospy
import copy, math

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random


GROUP_NAME_ARM = 'left_arm'
GROUP_NAME_GRIPPER = 'left_gripper'

GRIPPER_FRAME = 'l_gripper_link'

FIXED_FRAME = 'base_footprint'

GRIPPER_CLOSED = 0.3
GRIPPER_OPEN = 0.0

GRIPPER_JOINT_NAMES = ['l_wrist_roll_joint']

GRIPPER_EFFORT = [1.0]


class TestPick():
    def __init__(self):

        roscpp_initialize(sys.argv)
        rospy.init_node('moveit1_py_demo', anonymous=True)
       
        scene = PlanningSceneInterface()
        robot = RobotCommander()
        
        left_arm = MoveGroupCommander(GROUP_NAME_ARM)
        human_arm = MoveGroupCommander('human_right_arm')
        left_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        eef = left_arm.get_end_effector_link()
        human_eef = human_arm.get_end_effector_link()
        
        rospy.sleep(2)
        
    
        rospy.sleep(1)
    
        human_arm.set_random_target()
        plan1 = human_arm.plan()
        
        human_arm.execute(plan1)
        
        
        rospy.sleep(2)
        
    

        
        
        
        rospy.spin()
        roscpp_shutdown()
                  
        '''        # generate a list of grasps
        grasps = self.make_grasps(start_pose)
    
        result = False
        n_attempts = 0
        
        # repeat until will succeed
        while result == False:
            result = robot.right_arm.pick("part", grasps)       
            n_attempts += 1
            print "Attempts: ", n_attempts
            rospy.sleep(0.2)
            '''
           
        
        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, pose):
        t = JointTrajectory()
        t.joint_names = GRIPPER_JOINT_NAMES
        tp = JointTrajectoryPoint()
        tp.positions = [pose for j in t.joint_names]
        tp.effort = GRIPPER_EFFORT
        t.points.append(tp)
        return t
    
    def make_gripper_translation(self, min_dist, desired, axis=1.0):
        g = GripperTranslation()
        g.direction.vector.x = axis
        g.direction.header.frame_id = GRIPPER_FRAME
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_grasps(self, pose_stamped, mega_angle=False):
        # setup defaults for the grasp
        g = Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1)
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, -1.0)
        g.grasp_pose = pose_stamped
    
        pitch_vals = [0, 0.2, -0.2, 0.4, -0.4]
        #pitch_vals = [0]
        
        yaw_vals = [-0.2, -0.1, 0, 0.1, 0.2]
        #yaw_vals = [0]
        
        if mega_angle:
            pitch_vals += [0.78, -0.78, 0.3, -0.3, 0.5, -0.5, 0.6, -0.6]
    
        # generate list of grasps
        grasps = []
        for y in [-1.57, -0.78, 0, 0.78, 1.57]:

            for y in yaw_vals:
                for p in pitch_vals:
                    q = quaternion_from_euler(0, 1.57-p, y)
                    g.grasp_pose.pose.orientation.x = q[0]
                    g.grasp_pose.pose.orientation.y = q[1]
                    g.grasp_pose.pose.orientation.z = q[2]
                    g.grasp_pose.pose.orientation.w = q[3]
                    g.id = str(len(grasps))
                    g.allowed_touch_objects = ["part"]
                    g.max_contact_force = 0
                    #g.grasp_quality = 1.0 - abs(p/2.0)
                    grasps.append(copy.deepcopy(g))
        return grasps


if __name__=='__main__':
    TestPick()
