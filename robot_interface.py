#!/usr/bin/env python
# license removed for brevity
#from beginner_tutorials.srv import * TODO need to replace
import rospy
from std_msgs.msg import String



def robot_interface():
        pub = rospy.Publisher('chatter', String, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store
        
        config = rospy.get_param('/configuration')
        num_blocks = rospy.get_param('/num_blocks')

        /move_robot
        rospy.init_node('robot_interface', anonymous=True)
        rate = rospy.Rate(1) # 1 hz like instructions say

        open_gripper = "open_gripper"
        close_gripper = "close_gripper"
        move_to_block = "move_to_block" #meaning the Fingers are around a block.
        move_over_block = move_over_block
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()


        s = rospy.Service('move_robot', AddTwoInts, handle_move_robot) # /move_robot
        print "Ready to add two ints."
        rospy.spin()

def handle_move_robot(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))

    if req.action == open_gripper
        return 
    elif req.action == close_gripper

    elif req.action == move_to_block

    elif req.action == move_over_block

    else
        #invalid action

    return AddTwoIntsResponse(req.a + req.b)

def handle_get_world_state():

    #position on ( arm x, 0 , arm z) or something similar
    

    if config == "stacked_ascending"
        #position on ( arm x, 0 , arm z) or something similar
        #position on ( arm x, block_size , arm z) or something similarf

    elif config == "stack_descending"
    
    elif config == "scattered"
        #should not happen
    else
        #exit

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
