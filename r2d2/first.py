#!/usr/bin/python

import rospy
import roslib
import tf 
import geometry_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from std_msgs.msg import String
import numpy as np 
 
 
mapping = {'torso':'torso_','head_1':'head_', 'neck_1':'neck_', 'right_shoulder_1':'right_shoulder_' , 'right_elbow_1':'right_elbow_', 'right_hand_1':'right_hand_','left_shoulder_1':'left_shoulder_', 'left_elbow_1':'left_elbow_','left_hand_1':'left_hand_', 'left_hip_1':'left_hip_', 'left_knee_1':'left_knee_','left_foot_1':'left_foot_','right_hip_1':'right_hip_', 'right_knee_1':'right_knee_','right_foot_1':'right_foot_'}

joint_state=[]


def map_to_character():
    pub = rospy.Publisher("joint_states", JointState, queue_size = 10)
    rospy.set_param('~Fre',100)
    rate=rospy.Rate(rospy.get_param('~Fre'))
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        
        for i in range (1,10):
            torso_id= "torso_"+str(i)
            if listener.frameExists(torso_id):
                ID=str(i)
                break
            else:
                ID=str(1)
   
        try:     
         #listener.waitForTransform('neck_'+ID,'torso_'+ID,now, rospy.Duration(4.0))
         
             (trans1,rot1)=listener.lookupTransform('neck_'+ID, 'torso_'+ID, now)
             #joint_state.append(('neck_1',rot1))       
                                           
             
             (trans2,rot2)=listener.lookupTransform('head_'+ID, 'neck_'+ID, now)
             #joint_state.append(('head_1',rot2))       
             
             
             (trans3,rot3)=listener.lookupTransform('torso_'+ID, 'left_shoulder_'+ID, now)
             #joint_state.append(('left_shoulder_1',rot3))       
             
             
             (trans4,rot4)=listener.lookupTransform('left_shoulder_'+ID, 'left_elbow_'+ID, now)
             #joint_state.append(('left_elbow_1',rot4))       
             
             
             (trans5,rot5)=listener.lookupTransform('left_elbow_'+ID, 'left_hand_'+ID, now)
             #joint_state.append(('left_hand_1',rot5))       
             
             
             (trans6,rot6)=listener.lookupTransform('torso_'+ID, 'right_shoulder_'+ID, now)
             #joint_state.append(('right_shoulder_1',rot6))       
             
             
             (trans7,rot7)=listener.lookupTransform('right_shoulder_'+ID, 'right_elbow_'+ID, now)
             #joint_state.append(('right_elbow_1',rot7))       
             
             
             (trans8,rot8)=listener.lookupTransform('right_elbow_'+ID, 'right_hand_'+ID, now)
             #joint_state.append(('right_hand_1',rot8))       
             
             
             (trans9,rot9)=listener.lookupTransform('torso_'+ID, 'left_hip_'+ID, now)
             #joint_state.append(('left_hip_1',rot9))       
             
             
             (trans10,rot10)=listener.lookupTransform('left_hip_'+ID, 'left_knee_'+ID, now)
             #joint_state.append(('left_knee_1',rot10))       
             
             
             (trans11,rot11)=listener.lookupTransform('left_knee_'+ID, 'left_foot_'+ID, now)
             #joint_state.append(('left_foot_1',rot11))       
             
             
             (trans12,rot12)=listener.lookupTransform('torso_'+ID, 'right_hip_'+ID, now)
             #joint_state.append(('right_hip_1',rot12))       
             
             
             (trans13,rot13)=listener.lookupTransform('right_hip_'+ID, 'right_knee_'+ID, now)
             #joint_state.append(('right_knee_1',rot13))       
             
             
             (trans14,rot14)=listener.lookupTransform('right_knee_'+ID, 'right_foot_'+ID, now)
             #joint_state.append(('right_foot_1',rot14))
             
      
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass 
        
        dishes = dict(joint_state)
        keys=dishes.viewkeys()
        values = dishes.viewvalues()
        
        js = JointState(name=list(keys),position=list(values))
        js.header.stamp = rospy.Time.now()
        
        pub.publish(js)
        rate.sleep()             

if __name__ == '__main__':
    try:
        rospy.init_node('map_to_char', anonymous = True)
        listener = tf.TransformListener()
        rospy.sleep(2.0)
        base_time = rospy.Time.now()
        map_to_character()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
