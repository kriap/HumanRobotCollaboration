# special-problem

INSTRUCTIONS FOR SKELETAL TRACKING :

Step 1 :  SET UP ROS ENVIRONMENT
source ./catkin_ws/devel/setup.bash

Step 2 : RUN ROS MASTER
roscore

Step 3 : LAUNCH Openni_Launch
roslaunch openni_launch openni.launch 
 

Step 4 : Launch skeleton_markers
roslaunch skeleton_markers skeleton.launch

Step 5: Launch Rviz Rviz
roslaunch r2d2 humanskel.launch 

→ Add robot model to Rviz
→ Change fixed frame to openni_depth_frame
→ Add /tf to the frame .

Step 6: Run Python script to obtain information about joint positions and orientations
rosun r2d2 listener.py


INSTRUCTIONS FOR PICK AND PLACE:

Step 1 :  SET UP ROS ENVIRONMENT
source ./catkin_ws/devel/setup.bash

Step 2 : RUN ROS MASTER
roscore

STEP 3: Launch the PR2 robot  
roslaunch hrc_moveit_generated demo.launch

STEP 4: Run Python script to visualize the 'pick' operation.
python simplepick.py
