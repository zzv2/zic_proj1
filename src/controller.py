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
    return block_stack[0] is 1 and all( [ block_stack[i] < block_stack[i+1] for i in range(len(block_stack))] ) and block_stack[n-1] is n

def is_stacked_descending(block_stack):
    n = rospy.get_param("num_blocks")
    return block_stack[0] is n and all( [block_stack[i] > block_stack[i+1] for i in range(len(block_stack))]) and block_stack[n-1] is 1

def is_scattered(table):
    n = rospy.get_param("num_blocks")
    return all( i in table for i in xrange(1,n+1))

def scatter():
    rospy.loginfo("Beginning to scatter blocks")
    n = rospy.get_param("num_blocks")
    rospy.loginfo("There are {0} blocks to deal with".format(n))

    while len(get_state().stack) > 0:
        rospy.loginfo("\nBeginnning remove block subroutine")
        rospy.loginfo("There are {0} blocks on the stack".format(len(get_state().stack)))
        rospy.loginfo("There are {0} blocks on the table".format(len(get_state().table)))
        current_block = get_state().stack[-1]
        rospy.loginfo("Beginning to take block {0} off of the stack".format(current_block))

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

    if sorted(get_state().stack) is get_state().stack:
        rospy.loginfo("Blocks already stacked ascending.\n")
        return True
    elif not is_scattered(get_state().table):
        rospy.loginfo("Blocks aren't scattered, beginning to scatter.\n")
        scatter()

    rospy.loginfo("Blocks should be scattered")

    for i in range(1,n+1):
        rospy.loginfo("Beginning to put block {0} on the stack".format(i))
        
        rospy.loginfo("{0} is the index we're going to look up".format(i-1))
        rospy.loginfo("This is the stack: {0}".format(get_state().stack))
        rospy.loginfo("Beginning to move hand to block {0}".format(i))
        if move_robot(MOVE_TO_BLOCK, i):
            rospy.loginfo("Successfully moved to block {0}".format(i))

            if move_robot(CLOSE_GRIPPER, i):
                rospy.loginfo("Successfully closed gripper around block {0}".format(i))
                if i == 1:
    		    action, target = MOVE_TO_STACK_BOTTOM, 1
                    rospy.loginfo("Determined that there are no blocks in current stack, moving to base of stack")
                else:
                    action, target = MOVE_OVER_BLOCK, (i-1)
                    rospy.loginfo("Determined that highest block currently in stack is {0}, moving over it".format(i-1))
                if move_robot(action, target):
                    rospy.loginfo("Successfully moved over stack")
                    if move_robot(OPEN_GRIPPER, i):
                        rospy.loginfo("Successfully deposited block {0} on stack.\n".format(i))
                    else:
                        rospy.logerr("Failed to place block on stack.")
                else:
                    rospy.logerr("Failed to move over stack.")
                    rospy.loginfo("Attempting to place block back on table")
                    if move_robot(MOVE_OVER_TABLE, i):
                        rospy.loginfo("Moved block back on table, retrying")
                    else:
                        rospy.logerr("Failed to reset block to table, exiting")
                        raise UnrecoverableWorldStateException("Failed to place block on stack and failed to move back to table, blocks in unknown configuration")
            else:
                rospy.logerr("Failed to close hand around block {0}".format(i))
        else:
            rospy.logerr("Failed to move to block {0}".format(i))


def stack_descending():
    n = rospy.get_param("num_blocks")
    rospy.loginfo("Beginning to stack blocks descending")

    if list(reversed(sorted(get_state().stack))) is get_state().stack:
        rospy.loginfo("Blocks already stacked ascending.\n")
        return True
    elif not is_scattered(get_state().table):
        rospy.loginfo("Blocks aren't scattered, beginning to scatter.\n")
        scatter()

    rospy.loginfo("Blocks should be scattered")

    for i in range(n, 0, -1):
        rospy.loginfo("Beginning to put block {0} on the stack".format(i))
        
        rospy.loginfo("{0} is the index we're going to look up".format(i-1))
        rospy.loginfo("This is the stack: {0}".format(get_state().stack))
        rospy.loginfo("Beginning to move hand to block {0}".format(i))
        if move_robot(MOVE_TO_BLOCK, i):
            rospy.loginfo("Successfully moved to block {0}".format(i))

            if move_robot(CLOSE_GRIPPER, i):
                rospy.loginfo("Successfully closed gripper around block {0}".format(i))
                if i == n:
    		    action, target = MOVE_TO_STACK_BOTTOM, n
                    rospy.loginfo("Determined that there are no blocks in current stack, moving to base of stack")
                else:
                    action, target = MOVE_OVER_BLOCK, (i+1)
                    rospy.loginfo("Determined that highest block currently in stack is {0}, moving over it".format(i-1))
                if move_robot(action, target):
                    rospy.loginfo("Successfully moved over stack")
                    if move_robot(OPEN_GRIPPER, i):
                        rospy.loginfo("Successfully deposited block {0} on stack.\n".format(i))
                    else:
                        rospy.logerr("Failed to place block on stack.")
                else:
                    rospy.logerr("Failed to move over stack.")
                    rospy.loginfo("Attempting to place block back on table")
                    if move_robot(MOVE_OVER_TABLE, i):
                        rospy.loginfo("Moved block back on table, retrying")
                    else:
                        rospy.logerr("Failed to reset block to table, exiting")
                        raise UnrecoverableWorldStateException("Failed to place block on stack and failed to move back to table, blocks in unknown configuration")
            else:
                rospy.logerr("Failed to close hand around block {0}".format(i))
        else:
            rospy.logerr("Failed to move to block {0}".format(i))


def respond_to_command(command):
    rospy.loginfo("Recieved Command.")
    try:
        if command == String("scatter"):
            rospy.loginfo("Command is \"scatter\"")
            scatter()
        elif command == String("stack_ascending"):
            rospy.loginfo("Command is \"stack_ascending\"")
            stack_ascending()
        elif command == String("stack_descending"):
            rospy.loginfo("Command is \"stack_descending\"")
            stack_descending()
        else:
             rospy.logerr("Recieved invalid command: {0}".format(command))
             return False
    except UnrecoverableWorldStateException as e:
        rospy.logerr("Command failed due to reason:\n {0}".format(e.value))
        return False
    return True

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
        
