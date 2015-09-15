#!/usr/bin/env python
# license removed for brevity
#from beginner_tutorials.srv import * TODO need to replace
import rospy
from std_msgs.msg import String

def robot_interface():
        rospy.init_node('robot_interface', anonymous=True)

        state_publisher = rospy.Publisher('/state', String, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store
        
        config = rospy.get_param('/configuration')
        num_blocks = rospy.get_param("num_blocks")
        state = "current state: [1], [2], [3]\n"
        rospy.loginfo(state)# prints to console
        rospy.loginfo("config: %s\n",config)# prints to console

        #/move_robot
        broadcast_rate = rospy.Rate(1) # 1 hz like instructions say
        while not rospy.is_shutdown():
            # hello_str = "hello world %s" % rospy.get_time()
            # rospy.loginfo(hello_str)# prints to console
            # pub.publish(hello_str)

            # publish state
            state_publisher.publish(state)

            broadcast_rate.sleep()


        s = rospy.Service('/move_robot', MoveRobot, handle_move_robot) # /move_robot
        print "Ready to add two ints."
        rospy.spin()

def handle_move_robot(req):

    success = True

    if req.action == "open_gripper" :
        print "opened gripper"
    elif req.action == "close_gripper" :
        print "closed gripper"

    elif req.action == "move_to_block" :
        print "Moved to block " + req.block

    elif req.action == "move_over_block" :
        print "Moved OVER to block " + req.block

    else :
        print "invalid action"

    return 0 #MoveRobotResponse(success)

def handle_get_world_state():

    #position on ( arm x, 0 , arm z) or something similar
    

    if config == "stacked_ascending" :
        print "stack ascending"
        #position on ( arm x, 0 , arm z) or something similar
        #position on ( arm x, block_size , arm z) or something similarf

    elif config == "stack_descending" :
        print "stack descending"
    elif config == "scattered" :
        print "scattered world"
        #should not happen
    else :
        print "handlegetworldstate invalid state exit"

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
