#! usr/bin/env python

import rospy
import math
import tf

BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]

if __name__ == '__main__':
    rospy.init_node('tf_listener',anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
    
        try:
            frames = []
            print "The based frame looked at is : {}".format(BASE_FRAME)	
            for frame in FRAMES:
                (trans,rot) = listener.lookupTransform(BASE_FRAME, "%s" %(frame), rospy.Time(0))
                frames.append(trans)
                print "The position of {} is : {}".format(frame,trans)
                 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        rate.sleep() 
