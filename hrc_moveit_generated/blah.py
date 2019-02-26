import rospy
from pr2_pick_and_place_demos.pick_and_place_manager import *
from object_manipulator.convert_functions import *

class SimplePickAndPlaceExample():

    def __init__(self):

        rospy.loginfo("initializing pick and place manager")
        self.papm = PickAndPlaceManager()
        rospy.loginfo("finished initializing pick and place manager")


    #pick up the nearest object to PointStamped target_point with whicharm 
    #(0=right, 1=left)
    def pick_up_object_near_point(self, target_point, whicharm):
        
        rospy.loginfo("moving the arms to the side")
        self.papm.move_arm_to_side(0)  #right arm
        self.papm.move_arm_to_side(1)  #left arm

        rospy.loginfo("pointing the head at the target point")
        self.papm.point_head(get_xyz(target_point.point),
                             target_point.header.frame_id)

        rospy.loginfo("detecting the table and objects")
        self.papm.call_tabletop_detection(update_table = 1, update_place_rectangle = 1, 
                             clear_attached_objects = 1)     

        rospy.loginfo("picking up the nearest object to the target point")
        success = self.papm.pick_up_object_near_point(target_point, 
                                                      whicharm)
        
        if success:
            rospy.loginfo("pick-up was successful!  Moving arm to side")
            self.papm.move_arm_to_side(whicharm)
        else:
            rospy.loginfo("pick-up failed.")

        return success

    
    #place the object held in whicharm (0=right, 1=left) down in the 
    #place rectangle defined by place_rect_dims (x,y) 
    #and place_rect_center (PoseStamped)
    def place_object(self, whicharm, place_rect_dims, place_rect_center):

        self.papm.set_place_area(place_rect_center, place_rect_dims)
        
        rospy.loginfo("putting down the object in the %s gripper"\
                      %self.papm.arm_dict[whicharm])
        success = self.papm.put_down_object(whicharm, 
                      max_place_tries = 25,
                      use_place_override = 1)

        if success:
            rospy.loginfo("place returned success")
        else:
            rospy.loginfo("place returned failure")

        return success


if __name__ == "__main__":
    rospy.init_node('simple_pick_and_place_example')
    sppe = SimplePickAndPlaceExample()

    #adjust for your table 
    table_height = .72                                          

    #.5 m in front of robot, centered
    target_point_xyz = [.5, 0, table_height-.05]                
    target_point = create_point_stamped(target_point_xyz, 'base_link')
    success = sppe.pick_up_object_near_point(target_point, 0)   #right arm

    if success:

        #square of size 30 cm by 30 cm
        place_rect_dims = [.3, .3]                              

        #.5 m in front of robot, to the right
        center_xyz = [.5, -.15, table_height-.05]               

        #aligned with axes of frame_id
        center_quat = [0,0,0,1]                                 
        place_rect_center = create_pose_stamped(center_xyz+center_quat,
                                                    'base_link')

        sppe.place_object(0, place_rect_dims, place_rect_center)
