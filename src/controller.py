#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from zic_proj1.srv import MoveRobot, GetState
from config import * 

def scatter():
    rospy.loginfo("Beginning to scatter blocks")
    n = rospy.get_param("num_blocks")

    while len(get_state().stack) > 0:
        current_block = get_state().stack[-1]
        rospy.loginfo("\nBeginnning remove block subroutine for block {0}".format(current_block))
        rospy.loginfo("There are {0} blocks on the stack".format(len(get_state().stack)))
        rospy.loginfo("There are {0} blocks on the table".format(len(get_state().table)))
        rospy.loginfo("Beginning to take block {0} off of the stack".format(current_block))

        rospy.loginfo("Beginning to move hand to block {0}".format(current_block))
        move_robot(MOVE_TO_BLOCK, current_block)
        rospy.loginfo("Successfully moved hand to block {0}".format(current_block))

        rospy.loginfo("Beginning to close gripper around block {0}".format(current_block))
        move_robot(CLOSE_GRIPPER, current_block)
        rospy.loginfo("Successfully closed gripper around block {0}".format(current_block))

        rospy.loginfo("Begining to move gripper over table position for block {0}".format(current_block))
        move_robot(MOVE_OVER_TABLE, current_block)
        rospy.loginfo("Successfully moved gripper over table position for block {0}".format(current_block))

        rospy.loginfo("Beginning to open gripper to release block {0} onto table.".format(current_block))
        move_robot(OPEN_GRIPPER, 0)
        rospy.loginfo("Successfully deposited block {0} at its position on table".format(current_block))
    
    rospy.loginfo("\nSuccessfully scattered blocks.\n\n")

def stack_ascending():
    n = rospy.get_param("num_blocks")
    rospy.loginfo("Beginning to stack blocks ascending")

    if sorted(get_state().stack) is get_state().stack:
        rospy.loginfo("Blocks already stacked ascending.\n")
        return True
    elif not is_scattered(get_state().table):
        rospy.loginfo("Blocks aren't scattered, beginning to scatter.\n")
        scatter()
        rospy.loginfo("Successfully called Scatter subroutine, continuing to stack ascending.")

    for i in range(1,n+1):
        rospy.loginfo("\nBeginning Stack Subroutine for block {0}".format(i))
        rospy.loginfo("Beginning to put block {0} on the stack".format(i))

        rospy.loginfo("Beginning to move to block {0}".format(i))
        move_robot(MOVE_TO_BLOCK, i)
        rospy.loginfo("Moved to block {0}".format(i))

        rospy.loginfo("Beginning to close gripper around block {0}".format(i))
        move_robot(CLOSE_GRIPPER, i)
        rospy.loginfo("Closed gripper around block {0}".format(i))

        if i is 1:
            rospy.loginfo("Moving block {0} to base of stack".format(i))
            move_robot(MOVE_TO_STACK_BOTTOM, 1)
            rospy.loginfo("Moved block {0} to base of stack".format(i))
        else:
            rospy.loginfo("Moving block {0} above block {1}, on top of stack".format(i, i-1))
            move_robot(MOVE_OVER_BLOCK, i-1)
            rospy.loginfo("Moved block {0} over block {1}, block {0} is on top of stack".format(i,i-1))
        
        rospy.loginfo("Releasing gripper around block {0}".format(i))
        move_robot(OPEN_GRIPPER, i-1)
        rospy.loginfo("Released block {0} from gripper".format(i))
    
    rospy.loginfo("\nSuccessfully stacked blocks ascending.\n\n")

def stack_ascending():
    n = rospy.get_param("num_blocks")
    rospy.loginfo("Beginning to stack blocks descending")

    if list(reversed(sorted(get_state().stack))) is get_state().stack:
        rospy.loginfo("Blocks already stacked ascending.\n")
        return True
    elif not is_scattered(get_state().table):
        rospy.loginfo("Blocks aren't scattered, beginning to scatter.\n")
        scatter()
        rospy.loginfo("Successfully called Scatter subroutine, continuing to stack descending.")

    for i in range(n, 0, -1):
        rospy.loginfo("\nBeginning Stack Subroutine for block {0}".format(i))
        rospy.loginfo("Beginning to put block {0} on the stack".format(i))

        rospy.loginfo("Beginning to move to block {0}".format(i))
        move_robot(MOVE_TO_BLOCK, i)
        rospy.loginfo("Moved to block {0}".format(i))

        rospy.loginfo("Beginning to close gripper around block {0}".format(i))
        move_robot(CLOSE_GRIPPER, i)
        rospy.loginfo("Closed gripper around block {0}".format(i))

        if i is n:
            rospy.loginfo("Moving block {0} to base of stack".format(i))
            move_robot(MOVE_TO_STACK_BOTTOM, n)
            rospy.loginfo("Moved block {0} to base of stack".format(i))
        else:
            rospy.loginfo("Moving block {0} above block {1}, on top of stack".format(i, i+1))
            move_robot(MOVE_OVER_BLOCK, i+1)
            rospy.loginfo("Moved block {0} over block {1}, block {0} is on top of stack".format(i,i+1))
        
        rospy.loginfo("Releasing gripper around block {0}".format(i))
        move_robot(OPEN_GRIPPER, i+1)
        rospy.loginfo("Released block {0} from gripper".format(i))
    
    rospy.loginfo("\nSuccessfully stacked blocks descending.\n\n")


def respond_to_command(command):
    rospy.loginfo("Recieved Command.")
    if command == String("scatter"):
        rospy.loginfo("Command is \"scatter\"")
        scatter()
        rospy.loginfo("Executed command \"scatter\"")
    elif command == String("stack_ascending"):
        rospy.loginfo("Command is \"stack_ascending\"")
        stack_ascending()
        rospy.loginfo("Executed command \"stack_ascending\"")
    elif command == String("stack_descending"):
        rospy.loginfo("Command is \"stack_descending\"")
        stack_descending()
        rospy.loginfo("Executed command \"stack_descending\"")
    else:
        rospy.logerr("Recieved invalid command: {0}".format(command))

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    #rospy.loginfo("Initializing node Controller")
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
        rospy.logerr("Service Call Failed: {0}".format(e))


if __name__ == '__main__':
    listener()
        
