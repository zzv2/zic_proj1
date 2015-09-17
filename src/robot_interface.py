#!/usr/bin/env python
# license removed for brevity
#from beginner_tutorials.srv import * TODO need to replace
import rospy
from std_msgs.msg import String
from zic_proj1.msg import State
from zic_proj1.srv import MoveRobot

CLOSE_GRIPPER = 0
OPEN_GRIPPER = 1
MOVE_TO_BLOCK = 2
MOVE_OVER_BLOCK = 3
MOVE_OVER_TABLE = 4

#locations on table will be given by function in this file

def robot_interface():
        rospy.init_node('robot_interface', anonymous=True)

        state_publisher = rospy.Publisher('/state', State, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store
        
        config = rospy.get_param('/configuration')
        num_blocks = rospy.get_param("num_blocks")
        # state = "current state: [1], [2], [3]\n"

        state = State()
        state.gripper_closed = False
        state.block_in_gripper = 0
        if config == "stacked_ascending" :
            state.stack = range(1, num_blocks)
            state.table = [0] * num_blocks
        else :
            state.stack = range(num_blocks, 1)
            state.table = [0] * num_blocks #int32[num_blocks]

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
        print "Ready to move robot."
        rospy.spin()

def handle_move_robot(req):

    success = True

    if req.action == OPEN_GRIPPER :
        print "opened gripper"
        #Gripper.open ==> physical robot commands go here
    elif req.action == CLOSE_GRIPPER :
        print "closed gripper"

    elif req.action == MOVE_TO_BLOCK :
        print "Moved to block " + req.target

    elif req.action == MOVE_OVER_BLOCK :
        print "Moved OVER to block " + req.target
    elif req.action == MOVE_OVER_TABLE :
        print "Moving over table"
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
