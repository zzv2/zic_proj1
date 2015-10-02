iaq3 zzv2 cps232


Robot Manipulation Project 1 Readme


With launch file:


roscore


cd ~/ros_ws
. baxter.sh
roslaunch zic_proj1 proj1.launch


cd ~/ros_ws
. baxter.sh
----enter commands below


Without launch file:
 
roscore


cd ~/ros_ws
. baxter.sh
rosparam set num_blocks 4                                 (1-10)
rosparam set configuration “stacked_ascending”         (or “stacked_descending”)
rosparam set environment “simulator”                        (or “robot” or “symbolic”)
rosparam set limb “right”                                (or “left”)
rosparam set num_arms 1                                (or 2)
rosrun zic_proj1 robot_interface.py
rosrun zic_proj1 controller.py


Two arm mode:
If an odd block is on the top of the stack, then the right arm starts above the stack
If an even block is on the top of the stack, then the left arm starts above the stack
rosparam limb = the arm that starts above the stack


cd ~/ros_ws
. baxter.sh 
----enter commands below
Once running, run the following commands :
run the following commands from the terminal
rostopic pub -1 /command std_msgs/String "scatter"
rostopic pub -1 /command std_msgs/String "stacked_ascending"
rostopic pub -1 /command std_msgs/String "stacked_descending"
rostopic pub -1 /command std_msgs/String "hanoi" 
rostopic pub -1 /command std_msgs/String "odd_even"