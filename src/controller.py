#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from zic_proj1.srv import MoveRobot, GetState
from config import * 

class UnrecoverableWorldStateException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return self.value

def is_stacked_ascending(block_stack):
    n = rospy.get_param("num_blocks")
    return block_stack[0] is 1 and all( block_stack[i] < block_stack[i+1] for i in xrange(len(block_stack)-1)) and block_stack[n-1] is n

def is_stacked_descending(block_stack):
    n = rospy.get_param("num_blocks")
    return block_stack[0] is n and all( block_stack[i] > block_stack[i+1] for i in xrange(len(block_stack)-1)) and block_stack[n-1] is 1

def is_scattered(table):
    n = rospy.get_param("num_blocks")
    return all( i in table for i in xrange(1,n+1))

def scatter():
    rospy.loginfo("Beginning to scatter blocks")
    n = rospy.get_param("num_blocks")
    rospy.loginfo("There are {0} blocks on the table".format(n))

    while len(get_state().stack) > 0:
        rospy.loginfo("There are {0} blocks on the stack".format(len(get_state().stack)))
        current_block = get_state().stack[-1]
        rospy.loginfo("Beginning to take block {0} off of the stack".format(n))

        rospy.loginfo("Beginning to move hand to block {0}".format(current_block))
        if move_robot(MOVE_TO_BLOCK, current_block):
            rospy.loginfo("Successfully moved hand to block {0}".format(current_block))

            if move_robot(CLOSE_GRIPPER, current_block):
                rospy.loginfo("Successfully closed gripper around block {0}".format(current_block))

                if move_robot(MOVE_OVER_TABLE, current_block):
                    rospy.loginfo("Successfully moved gripper over table position for block {0}".format(current_block))

                    if move_robot(OPEN_GRIPPER, 0):
                        rospy.loginfo("Successfully deposited block {0} at its position on table.\n".format(current_block))
                    else:
                        rospy.logerr("Failed to let go of block {0}.".format(current_block))
                        raise UnrecoverableWorldStateException("Dislodged block {0} on table".format(current_block))
                else:
                    rospy.logerr("Failed to move block {0} to its position on table".format(current_block))
                    raise UnrecoverableWorldStateException("Dropped block on the way to table")
            else:
                rospy.logerr("Failed to close gripper around block {0}, retrying".format(current_block))
        else:
            rospy.logerr("Failed to move hand to block {0}, retrying".format(current_block))

    rospy.loginfo("Successfully scattered blocks.\n\n")
    return True

def stack_ascending():
    n = rospy.get_param("num_blocks")
    rospy.loginfo("Beginning to stack blocks ascending")

    if is_stacked_ascending(get_state().stack):
        rospy.loginfo("Blocks already stacked ascending.\n")
        return True
    elif not is_scattered(get_state().table):
        rospy.loginfo("Blocks aren't scattered, beginning to scatter.\n")
        scatter()

    rospy.loginfo("Blocks should be scattered")

    for i in xrange(1, n+1):
        rospy.loginfo("Beginning to put block {0} on the stack".format(i))
        

        while get_state().stack[i-1] != i:
            rospy.loginfo("Beginning to move hand to block {0}".format(i))
            if move_robot(MOVE_TO_BLOCK, i):
                rospy.loginfo("Successfully moved to block {0}".format(i))
    
                if move_robot(CLOSE_GRIPPER, i):
                    rospy.loginfo("Successfully closed gripper around block {0}".format(i))
    
                    """ WE NEED A WAY TO EXPRESS MOVING TO THE BASE OF THE STACK """
                else:
                    rospy.logerr("Failed to close hand around block {0}".format(i))
            else:
                rospy.logerr("Failed to move to block {0}".format(i))




def respond_to_command(command):
    if command == String("scatter"):
        rospy.loginfo("Received Command: scatter")

        if is_scattered(get_state().table):
            rospy.loginfo("Blocks are already in state scattered")
        else:
            while len(get_state().stack) > 0:
                current_block = get_state().stack[-1]
                move_robot(MOVE_TO_BLOCK, current_block) 
                move_robot(CLOSE_GRIPPER, current_block)
                move_robot(MOVE_OVER_TABLE, current_block)
                move_robot(OPEN_GRIPPER, 0)
            
    elif command == String("stack_ascending"):
        rospy.loginfo("Received Command: stack_ascending")

        if is_stacked_ascending(get_state().stack):
            rospy.loginfo("Blocks are already in state stack_ascending")
        else:
            pass

    elif command == String("stack_descending"):
        rospy.loginfo("Received Command: stack_descending")

        if is_stacked_descending(get_state().stack):
            rospy.loginfo("Blocks are already in state stack_descending")
        else:
            pass
    else:
        rospy.logerr("Received Malformed Command")

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    rospy.init_node('controller')
    rospy.loginfo("Initialized node Controller")

    # Initialize the /move_robot service from robot_interface
    rospy.loginfo("Beginning to wait for service /move_robot...")
    rospy.wait_for_service("/move_robot")
    rospy.loginfo("Service /move_robot now available.")

    # Initialize the /get_state service from robot_interface
    rospy.loginfo("Beginning to wait for service /get_state...")
    rospy.wait_for_service("/get_state")
    rospy.loginfo("Service /get_state now available.")
    try:
        # Initialize the service proxy for the /move_robot server
        rospy.loginfo("Initializing service proxy for /move_robot...")
        global move_robot
	move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
        rospy.loginfo("Initialized service proxy for /move_robot")

        # Initialize the service proxy for the /get_state server
        rospy.loginfo("Initializing service proxy for /get_state...")
        global get_state
        get_state = rospy.ServiceProxy("/get_state", GetState)
        rospy.loginfo("Initialized service proxy for /get_state...")
    
        # the controller subscribes to the /command topic and listens for commands
        rospy.loginfo("Subscribing to topic /command")
        rospy.Subscriber("command", String, respond_to_command)
        rospy.loginfo("Subscribed to topic /command")

        # keep the server open
        rospy.spin()

    except rospy.ServiceException, e:
        rospy.logerr("Service Call Failed")





if __name__ == '__main__':
    listener()
        
