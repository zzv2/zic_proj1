#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from zic_proj1.srv import MoveRobot, GetState
from config import * 
from threading import Lock

right_lock = Lock()
left_lock = Lock()

def log_info(log_string):
    rospy.loginfo("Controller: {0}".format(log_string))

def log_err(log_string):
    rospy.logerr("Controller: {0}".format(log_string))

def MoveTower(block, source, dest, spare) :
    if block == 1 :
        #move from source to dest
        MoveBlock(block, source, dest)
    else :
        MoveTower(block - 1, source, spare, dest)
        #move from source to dest
        MoveBlock(block, source, dest)
        MoveTower(block -1, spare, dest, source)

def MoveBlock(block, source, dest) :
    if source == 1 :
        move_robot(MOVE_TO_LEFT_TOWER, block)
    elif source == 2 :
        move_robot(MOVE_TO_MID_TOWER, block)
    else :
        move_robot(MOVE_TO_RIGHT_TOWER, block)

    move_robot(CLOSE_GRIPPER_HANOI, block)

    if dest == 1 :
        move_robot(MOVE_TO_ABOVE_LEFT_TOWER, block)
    elif dest == 2 :
        move_robot(MOVE_TO_ABOVE_MID_TOWER, block)
    else :
        move_robot(MOVE_TO_ABOVE_RIGHT_TOWER, block)

    move_robot(OPEN_GRIPPER_HANOI, -2) #

def scatter():
    num_arms = rospy.get_param("num_arms")
    limb = rospy.get_param("limb")
    n = rospy.get_param("num_blocks")

    move_robot = move_robot_left if limb == "left" else move_robot_right
    lock = left_lock if limb == "left" else right_lock

    log_info("Beginning to scatter blocks")

    while len(get_state().stack) > 0:
        current_block = get_state().stack[-1]
        log_info("\nBeginnning remove block subroutine for block {0}".format(current_block))
        log_info("There are {0} blocks on the stack".format(len(get_state().stack)))
        log_info("There are {0} blocks on the table".format(len(get_state().table)))
        log_info("Beginning to take block {0} off of the stack".format(current_block))

        if len(get_state().table) == 0:
            log_info("Robot is in starting configuration. Skipping Directly to CLOSE_GRIPPER")
        else:
            log_info("Beginning to move hand to block {0}".format(current_block))
            lock.acquire()
            move_robot(MOVE_TO_BLOCK, current_block)
            log_info("Successfully moved hand to block {0}".format(current_block))

        log_info("Beginning to close gripper around block {0}".format(current_block))
        lock.acquire()
        move_robot(CLOSE_GRIPPER, current_block)
        log_info("Successfully closed gripper around block {0}".format(current_block))

        log_info("Begining to move gripper over table position for block {0}".format(current_block))
        lock.acquire()
        move_robot(MOVE_OVER_TABLE, current_block)
        log_info("Successfully moved gripper over table position for block {0}".format(current_block))

        log_info("Beginning to open gripper to release block {0} onto table.".format(current_block))
        lock.acquire()
        move_robot(OPEN_GRIPPER, -1)
        lock.acquire()
        move_robot(WAIT_FOR_DONE, 0)
        log_info("Successfully deposited block {0} at its position on table".format(current_block))
    
    log_info("\nSuccessfully scattered blocks.\n\n")

def stack_ascending():
    num_arms = rospy.get_param("num_arms")
    limb = rospy.get_param("limb")
    n = rospy.get_param("num_blocks")

    move_robot = move_robot_left if limb == "left" else move_robot_right
    lock = left_lock if limb == "left" else right_lock

    log_info("Beginning to stack blocks ascending")

    if sorted(get_state().stack) is get_state().stack:
        log_info("Blocks already stacked ascending.\n")
        return True
    elif len(get_state().stack) > 0:
        log_info("Blocks aren't scattered, beginning to scatter.\n")
        scatter()
        log_info("Successfully called Scatter subroutine, continuing to stack ascending.")

    for i in range(1,n+1):
        log_info("\nBeginning Stack Subroutine for block {0}".format(i))
        log_info("Beginning to put block {0} on the stack".format(i))

        log_info("Beginning to move to block {0}".format(i))
        lock.acquire()
        move_robot(MOVE_TO_BLOCK, i)
        log_info("Moved to block {0}".format(i))

        log_info("Beginning to close gripper around block {0}".format(i))
        lock.acquire()
        move_robot(CLOSE_GRIPPER, i)
        log_info("Closed gripper around block {0}".format(i))

        if i is 1:
            log_info("Moving block {0} to base of stack".format(i))
            lock.acquire()
            move_robot(MOVE_TO_STACK_BOTTOM, 1)
            log_info("Moved block {0} to base of stack".format(i))
        else:
            log_info("Moving block {0} above block {1}, on top of stack".format(i, i-1))
            lock.acquire()
            move_robot(MOVE_OVER_BLOCK, i-1)
            log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(i,i-1))
        
        log_info("Releasing gripper around block {0}".format(i))
        lock.acquire()
        move_robot(OPEN_GRIPPER, i-1)
        lock.acquire()
        move_robot(WAIT_FOR_DONE, 0)
        log_info("Released block {0} from gripper".format(i))
    
    log_info("\nSuccessfully stacked blocks ascending.\n\n")

def stack_descending():
    num_arms = rospy.get_param("num_arms")
    limb = rospy.get_param("limb")
    n = rospy.get_param("num_blocks")

    move_robot = move_robot_left if limb == "left" else move_robot_right
    lock = left_lock if limb == "left" else right_lock

    log_info("Beginning to stack blocks descending")

    if list(reversed(sorted(get_state().stack))) is get_state().stack:
        log_info("Blocks already stacked ascending.\n")
        return True
    elif len(get_state().stack) > 0:
        log_info("Blocks aren't scattered, beginning to scatter.\n")
        scatter()
        log_info("Successfully called Scatter subroutine, continuing to stack descending.")

    for i in range(n, 0, -1):
        log_info("\nBeginning Stack Subroutine for block {0}".format(i))
        log_info("Beginning to put block {0} on the stack".format(i))

        log_info("Beginning to move to block {0}".format(i))
        lock.acquire()
        move_robot(MOVE_TO_BLOCK, i)
        log_info("Moved to block {0}".format(i))

        log_info("Beginning to close gripper around block {0}".format(i))
        lock.acquire()
        move_robot(CLOSE_GRIPPER, i)
        log_info("Closed gripper around block {0}".format(i))

        if i is n:
            log_info("Moving block {0} to base of stack".format(i))
            lock.acquire()
            move_robot(MOVE_TO_STACK_BOTTOM, n)
            log_info("Moved block {0} to base of stack".format(i))
        else:
            log_info("Moving block {0} above block {1}, on top of stack".format(i, i+1))
            lock.acquire()
            move_robot(MOVE_OVER_BLOCK, i+1)
            log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(i,i+1))
        
        log_info("Releasing gripper around block {0}".format(i))
        lock.acquire()
        move_robot(OPEN_GRIPPER, i+1)
        lock.acquire()
        move_robot(WAIT_FOR_DONE, 0)
        log_info("Released block {0} from gripper".format(i))
    
    log_info("\nSuccessfully stacked blocks descending.\n\n")

def scatter_parallel(): # left arm must start if even on top, right arm if odd on top
    num_arms = rospy.get_param("num_arms")
    if not num_arms == 2:
        raise Exception("num_arms needs to be 2 for scatter_parallel")
    limb = rospy.get_param("limb")
    n = rospy.get_param("num_blocks")
    configuration = rospy.get_param("configuration")

    log_info("Beginning to stack blocks scatter_parallel")

    global left_lock
    global right_lock
    
    top_two = get_state().stack[-2:]
    stack_len = len(get_state().stack)

    idx = stack_len - 1
    current_block1 = get_state().stack[idx]
    current_block2 = get_state().stack[idx-1]

    limb = "left" if current_block1 % 2 == 0 else "right"
    limb_other = "right" if current_block1 % 2 == 0 else "left"

    move_robot = move_robot_left if limb == "left" else move_robot_right
    move_robot_other = move_robot_right if limb == "left" else move_robot_left

    lock = left_lock if limb == "left" else right_lock
    lock_other = right_lock if limb == "left" else left_lock

    log_info("\nBeginnning remove block subroutine for block {0}".format(current_block1))
    log_info("There are {0} blocks on the stack".format(len(get_state().stack)))
    log_info("There are {0} blocks on the table".format(len(get_state().table)))
    log_info("Beginning to take block {0} off of the stack".format(current_block1))



    log_info("Robot is in starting configuration. Skipping Directly to CLOSE_GRIPPER")
    log_info("Beginning to close {0} gripper around block {1}".format(limb,current_block1))
    lock.acquire()
    move_robot(CLOSE_GRIPPER, current_block1)
    log_info("Successfully closed {0} gripper around block {1}".format(current_block1,limb))



    log_info("Begining to move {0} gripper over table position for block {1}".format(limb,current_block1))
    lock.acquire()
    move_robot(MOVE_OVER_TABLE, current_block1)
    log_info("Successfully moved {0} gripper over table position for block {1}".format(limb,current_block1))


    log_info("Beginning to move {0} hand to block {1}".format(limb_other,current_block2))
    lock_other.acquire()
    move_robot_other(MOVE_TO_BLOCK, current_block2)
    log_info("Successfully moved {0} hand to block {1}".format(limb_other,current_block2))




    log_info("Beginning to open {0} gripper to release block {1} onto table.".format(limb,current_block1))
    lock.acquire()
    move_robot(OPEN_GRIPPER, -1)
    log_info("Successfully {0} deposited block {1} at its position on table".format(limb,current_block1))


    log_info("Beginning to close {0} gripper around block {1}".format(limb_other,current_block2))
    lock_other.acquire()
    move_robot_other(CLOSE_GRIPPER, current_block2)
    log_info("Successfully closed {0} gripper around block {1}".format(limb_other,current_block2))


    idx -= 2
    while idx > 0:    
        current_block1 = get_state().stack[idx]


        log_info("Beginning to move {0} hand to block {1}".format(limb,current_block1))
        lock.acquire()
        move_robot(MOVE_TO_BLOCK, current_block1)
        log_info("Successfully moved {0} hand to block {1}".format(limb,current_block1))


        log_info("Begining to move {0} gripper over table position for block {1}".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(MOVE_OVER_TABLE, current_block2)
        log_info("Successfully moved {0} gripper over table position for block {1}".format(limb_other,current_block2))




        log_info("Beginning to close {0} gripper around block {1}".format(limb,current_block1))
        lock.acquire()
        move_robot(CLOSE_GRIPPER, current_block1)
        log_info("Successfully closed {0} gripper around block {1}".format(limb,current_block1))


        log_info("Beginning to open {0} gripper to release block {1} onto table.".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on table".format(limb_other,current_block2))


        current_block2 = get_state().stack[idx-1]


        log_info("Beginning to move {0} hand to block {1}".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(MOVE_TO_BLOCK, current_block2)
        log_info("Successfully moved {0} hand to block {1}".format(limb_other,current_block2))


        log_info("Begining to move {0} gripper over table position for block {1}".format(limb,current_block1))
        lock.acquire()
        move_robot(MOVE_OVER_TABLE, current_block1)
        log_info("Successfully moved {0} gripper over table position for block {1}".format(limb,current_block1))




        log_info("Beginning to close {0} gripper around block {1}".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(CLOSE_GRIPPER, current_block2)
        log_info("Successfully closed {0} gripper around block {1}".format(limb_other,current_block2))

        log_info("Beginning to open {0} gripper to release block {1} onto table.".format(limb,current_block1))
        lock.acquire()
        move_robot(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on table".format(limb,current_block1))

        idx -= 2
    
    lock.acquire()
    lock_other.acquire()
    remain = len(get_state().stack)
    # print "remain: %d" % remain
    lock.release()
    lock_other.release()

    if remain > 0:
        current_block1 = get_state().stack[idx]

        if remain > 1:
            log_info("Beginning to move {0} hand to block {1}".format(limb,current_block1))
            lock.acquire()
            move_robot(MOVE_TO_BLOCK, current_block1)
            log_info("Successfully moved {0} hand to block {1}".format(limb,current_block1))


        log_info("Begining to move {0} gripper over table position for block {1}".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(MOVE_OVER_TABLE, current_block2)
        log_info("Successfully moved {0} gripper over table position for block {1}".format(limb_other,current_block2))        

        if remain > 1:
            log_info("Beginning to close {0} gripper around block {1}".format(limb,current_block1))
            lock.acquire()
            move_robot(CLOSE_GRIPPER, current_block1)
            log_info("Successfully closed {0} gripper around block {1}".format(limb,current_block1))

        log_info("Beginning to open {0} gripper to release block {1} onto table.".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on table".format(limb_other,current_block2))


    if remain > 1:
        current_block2 = get_state().stack[idx-1]

        log_info("Begining to move {0} gripper over table position for block {1}".format(limb,current_block1))
        lock.acquire()
        move_robot(MOVE_OVER_TABLE, current_block1)
        log_info("Successfully moved {0} gripper over table position for block {1}".format(limb,current_block1))


        log_info("Beginning to open {0} gripper to release block {1} onto table.".format(limb,current_block1))
        lock.acquire()
        move_robot(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on table".format(limb,current_block1))

    log_info("\nSuccessfully stacked blocks scatter_parallel.\n\n")

def odd_even(): # left arm must start if even on top, right arm if odd on top
    num_arms = rospy.get_param("num_arms")
    if not num_arms == 2:
        raise Exception("num_arms needs to be 2 for odd_even")
    limb = rospy.get_param("limb")
    n = rospy.get_param("num_blocks")
    configuration = rospy.get_param("configuration")

    log_info("Beginning to stack blocks odd_even")

    global left_lock
    global right_lock
    
    top_two = get_state().stack[-2:]
    stack_len = len(get_state().stack)

    idx = stack_len - 1
    current_block1 = get_state().stack[idx]
    current_block2 = get_state().stack[idx-1]
    prev_block1 = 0
    prev_block2 = 0

    limb = "left" if current_block1 % 2 == 0 else "right"
    limb_other = "right" if current_block1 % 2 == 0 else "left"

    move_robot = move_robot_left if limb == "left" else move_robot_right
    move_robot_other = move_robot_right if limb == "left" else move_robot_left

    lock = left_lock if limb == "left" else right_lock
    lock_other = right_lock if limb == "left" else left_lock

    log_info("\nBeginnning remove block subroutine for block {0}".format(current_block1))
    log_info("There are {0} blocks on the stack".format(len(get_state().stack)))
    log_info("There are {0} blocks on the table".format(len(get_state().table)))
    log_info("Beginning to take block {0} off of the stack".format(current_block1))



    log_info("Robot is in starting configuration. Skipping Directly to CLOSE_GRIPPER")
    log_info("Beginning to close {0} gripper around block {1}".format(limb,current_block1))
    lock.acquire()
    move_robot(CLOSE_GRIPPER, current_block1)
    log_info("Successfully closed {0} gripper around block {1}".format(current_block1,limb))
    


    log_info("Begining to move {0} gripper over table position for block {1}".format(limb,current_block1))
    lock.acquire()
    move_robot(MOVE_OVER_TABLE, current_block1)
    log_info("Successfully moved {0} gripper over table position for block {1}".format(limb,current_block1))


    log_info("Beginning to move {0} hand to block {1}".format(limb_other,current_block2))
    lock_other.acquire()
    move_robot_other(MOVE_TO_BLOCK, current_block2)
    log_info("Successfully moved {0} hand to block {1}".format(limb_other,current_block2))



    log_info("Beginning to open {0} gripper to release block {1} onto table.".format(limb,current_block1))
    lock.acquire()
    move_robot(OPEN_GRIPPER, -1)
    log_info("Successfully {0} deposited block {1} at its position on table".format(limb,current_block1))


    log_info("Beginning to close {0} gripper around block {1}".format(limb_other,current_block2))
    lock_other.acquire()
    move_robot_other(CLOSE_GRIPPER, current_block2)
    log_info("Successfully closed {0} gripper around block {1}".format(limb_other,current_block2))


    idx -= 2
    while idx > 0:
        prev_block1 = current_block1
        current_block1 = get_state().stack[idx]


        log_info("Beginning to move {0} hand to block {1}".format(limb,current_block1))
        lock.acquire()
        move_robot(MOVE_TO_BLOCK, current_block1)
        log_info("Successfully moved {0} hand to block {1}".format(limb,current_block1))


        if prev_block2 == 0:
            log_info("Begining to move {0} gripper over table position for block {1}".format(limb_other,current_block2))
            lock_other.acquire()
            move_robot_other(MOVE_OVER_TABLE, current_block2)
            log_info("Successfully moved {0} gripper over table position for block {1}".format(limb_other,current_block2))
        else:
            log_info("Moving block {0} above block {1}, on top of stack".format(current_block2, prev_block2))
            lock_other.acquire()
            move_robot_other(MOVE_OVER_BLOCK, prev_block2)
            log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(current_block2,prev_block2))


        log_info("Beginning to close {0} gripper around block {1}".format(limb,current_block1))
        lock.acquire()
        move_robot(CLOSE_GRIPPER, current_block1)
        log_info("Successfully closed {0} gripper around block {1}".format(limb,current_block1))


        log_info("Beginning to open {0} gripper to release block {1} onto stack.".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on stack".format(limb_other,current_block2))


        prev_block2 = current_block2
        current_block2 = get_state().stack[idx-1]


        log_info("Beginning to move {0} hand to block {1}".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(MOVE_TO_BLOCK, current_block2)
        log_info("Successfully moved {0} hand to block {1}".format(limb_other,current_block2))


        log_info("Moving block {0} above block {1}, on top of stack".format(current_block1, prev_block1))
        lock.acquire()
        move_robot(MOVE_OVER_BLOCK, prev_block1)
        log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(current_block1,prev_block1))



        log_info("Beginning to close {0} gripper around block {1}".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(CLOSE_GRIPPER, current_block2)
        log_info("Successfully closed {0} gripper around block {1}".format(limb_other,current_block2))

        log_info("Beginning to open {0} gripper to release block {1} onto stack.".format(limb,current_block1))
        lock.acquire()
        move_robot(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on stack".format(limb,current_block1))

        idx -= 2
    
    lock.acquire()
    lock_other.acquire()
    remain = len(get_state().stack)
    # print "remain: %d" % remain
    lock.release()
    lock_other.release()

    if remain > 0:
        prev_block1 = current_block1
        current_block1 = get_state().stack[idx]

        if remain > 1:
            log_info("Beginning to move {0} hand to block {1}".format(limb,current_block1))
            lock.acquire()
            move_robot(MOVE_TO_BLOCK, current_block1)
            log_info("Successfully moved {0} hand to block {1}".format(limb,current_block1))


        log_info("Moving block {0} above block {1}, on top of stack".format(current_block2, prev_block2))
        lock_other.acquire()
        move_robot_other(MOVE_OVER_BLOCK, prev_block2)
        log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(current_block2,prev_block2))        

        if remain > 1:
            log_info("Beginning to close {0} gripper around block {1}".format(limb,current_block1))
            lock.acquire()
            move_robot(CLOSE_GRIPPER, current_block1)
            log_info("Successfully closed {0} gripper around block {1}".format(limb,current_block1))

        log_info("Beginning to open {0} gripper to release block {1} onto stack.".format(limb_other,current_block2))
        lock_other.acquire()
        move_robot_other(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on stack".format(limb_other,current_block2))


    if remain > 1:
        prev_block2 = current_block2
        current_block2 = get_state().stack[idx-1]

        log_info("Moving block {0} above block {1}, on top of stack".format(current_block1, prev_block1))
        lock.acquire()
        move_robot(MOVE_OVER_BLOCK, prev_block1)
        log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(current_block1,prev_block1))


        log_info("Beginning to open {0} gripper to release block {1} onto stack.".format(limb,current_block1))
        lock.acquire()
        move_robot(OPEN_GRIPPER, -1)
        log_info("Successfully {0} deposited block {1} at its position on stack".format(limb,current_block1))

    log_info("\nSuccessfully stacked blocks odd_even.\n\n")

def respond_to_command(command):
    log_info("Recieved Command.")
    num_arms = rospy.get_param("num_arms")

    if command == String("scatter"):
        log_info("Command is \"scatter\"")
        if num_arms == 1:
            scatter()
        if num_arms == 2:
            scatter_parallel()
        log_info("Executed command \"scatter\"")
    elif command == String("stack_ascending"):
        log_info("Command is \"stack_ascending\"")
        stack_ascending()
        log_info("Executed command \"stack_ascending\"")
    elif command == String("stack_descending"):
        log_info("Command is \"stack_descending\"")
        stack_descending()
        log_info("Executed command \"stack_descending\"")
    elif command == String("odd_even"):
        log_info("Command is \"odd_even\"")
        odd_even()
        log_info("Executed command \"odd_even\"")
    elif command == String("hanoi") :
        log_info("Command is hanoi")
        MoveTower( rospy.get_param("num_blocks"),1,3,2)
        log_info("Executed command \"hanoi\"")
    else:
        log_err("Recieved invalid command: {0}".format(command))

def right_lockCb(locked):
    global right_lock
    if locked.data:
        # print "#########################################"
        # print "right_lock locked"
        # print "#########################################"
        right_lock.acquire()
    else:
        # print "#########################################"
        # print "right_lock unlocked"
        # print "#########################################"
        right_lock.release()

def left_lockCb(locked):
    global left_lock
    if locked.data:
        # print "#########################################"
        # print "left_lock locked"
        # print "#########################################"
        left_lock.acquire()
    else:
        # print "#########################################"
        # print "left_lock unlocked"
        # print "#########################################"
        left_lock.release()

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    #log_info("Initializing node Controller")
    rospy.init_node('controller')
    log_info("Initialized node Controller")

    # Initialize the /move_robot service from robot_interface
    log_info("Beginning to wait for service /move_robot...")
    # rospy.wait_for_service("/move_robot")
    rospy.wait_for_service("/move_robot_left")
    rospy.wait_for_service("/move_robot_right")
    log_info("Service /move_robot now available.")

    # Initialize the /get_state service from robot_interface
    log_info("Beginning to wait for service /get_state...")
    rospy.wait_for_service("/get_state")
    log_info("Service /get_state now available.")
    try:
        # Initialize the service proxy for the /move_robot server
        log_info("Initializing service proxy for /move_robot...")
        # global move_robot
        global move_robot_left
        global move_robot_right
        # move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
        move_robot_left = rospy.ServiceProxy("/move_robot_left", MoveRobot)
        move_robot_right = rospy.ServiceProxy("/move_robot_right", MoveRobot)
        log_info("Initialized service proxy for /move_robot")

        # Initialize the service proxy for the /get_state server
        log_info("Initializing service proxy for /get_state...")
        global get_state
        get_state = rospy.ServiceProxy("/get_state", GetState)
        log_info("Initialized service proxy for /get_state...")

        # the controller subscribes to the /command topic and listens for commands
        log_info("Subscribing to topic /command")
        rospy.Subscriber("command", String, respond_to_command)
        log_info("Subscribed to topic /command")

        rospy.Subscriber("right_lock", Bool, right_lockCb)
        rospy.Subscriber("left_lock", Bool, left_lockCb)


        # keep the server open
        rospy.spin()

    except rospy.ServiceException, e:
        log_err("Service Call Failed: {0}".format(e))

    num_arms = rospy.get_param("num_arms")
    limb = rospy.get_param("limb")
    rospy.loginfo("num_arms: %d",num_arms)
    rospy.loginfo("limb: %s",limb)


if __name__ == '__main__':
    listener()
        
