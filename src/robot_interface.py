#!/usr/bin/env python
# license removed for brevity
#from beginner_tutorials.srv import * TODO need to replace
import rospy
from std_msgs.msg import String
from zic_proj1.msg import State
from zic_proj1.srv import MoveRobot
from zic_proj1.srv import GetState
from config import *

state = State()

#locations on table will be given by function in this file

def robot_interface():
        rospy.init_node('robot_interface', anonymous=True)

        state_publisher = rospy.Publisher('/state', State, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store
        
        move_robot_service = rospy.Service('/move_robot', MoveRobot, handle_move_robot) # /move_robot
        get_state_service = rospy.Service('/get_state', GetState, handle_get_world_state) # /move_robot

        print "Ready to move robot."

        config = rospy.get_param('configuration')
        num_blocks = rospy.get_param("num_blocks")

        state.gripper_closed = False
        state.block_in_gripper = 0
        state.stack = range(1, num_blocks)
        if config == "stacked_descending" :
            state.stack.reverse()
        state.table = []

        rospy.loginfo("configuration: %s",config)
        rospy.loginfo("num_blocks: %d",num_blocks)

        broadcast_rate = rospy.Rate(1) # 1 hz
        while not rospy.is_shutdown():
            # publish state
            state_publisher.publish(state)
            rospy.loginfo(state)
            broadcast_rate.sleep()

        
        rospy.spin()

def handle_move_robot(req):

    success = True

    if req.action == OPEN_GRIPPER : #CHRIS  we are using the target as the destination for this block
        #in practice this means calling MoveRobot -OPEN_GRIPPER with same target as Move_Robot - 
        #Move_over_block
        print "opened gripper"
        state.gripper_closed = False
        state.block_in_gripper = 0
        if req.target == 0 : #putting block on table
            state.table.append(req.target)
            del state.stack[state.stack.index(req.target)]
        else : #appending to stack
            state.stack.append(req.target)
            del state.table[state.table.index(req.target)]

        #Gripper.open ==> physical robot commands go here
    elif req.action == CLOSE_GRIPPER :
        print "closed gripper"
        state.gripper_closed = True
        state.block_in_gripper = req.target

    elif req.action == MOVE_TO_BLOCK :
        print "Moved to block " + req.target
        if state.block_in_gripper > 0 or state.gripper_closed:
            success = False

    elif req.action == MOVE_OVER_BLOCK :
        print "Moved over block " + req.target
    elif req.action == MOVE_OVER_TABLE :
        print "Moved over table"
    elif req.action == MOVE_TO_STACK_BOTTOM :
        print "Moved to stack bottom"
    else :
        print "invalid action"

    return MoveRobotResponse(success)

def handle_get_world_state(req):
    return GetStateResponse(state)

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
