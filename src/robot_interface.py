#!/usr/bin/env python
# license removed for brevity
#from beginner_tutorials.srv import * TODO need to replace
import rospy
from std_msgs.msg import * # String and header
from zic_proj1.msg import State
from zic_proj1.srv import *
from config import *
from geometry_msgs.msg import * # PoseStamped, quaternion, point, pose
from baxter_core_msgs.msg import * #(SolvePositionIK, SolvePositionIKRequest)
from baxter_core_msgs.srv import *
from baxter_interface import *
from copy import deepcopy
from threading import Thread, Lock

state = State()

hand_pose_right = Pose() #updated at 100 hz
hand_pose_left = Pose() #updated at 100 hz
initial_pose = Pose()

block_size = .045 #meters, 1.75 inches, .04445 meters
num_blocks = 0

table_z = 1 # <---find this

block_poses = [] #positioned at initialization, block 1 at index 0, block 2 at index 1 etc

left_tower = Pose() #*********************************************************HANOI 20
mid_tower = Pose()
right_tower = Pose()

left = 0
mid = 0
right = 0

active_tower = 1

MOVE_WAIT = 0.1
GRIPPER_WAIT = 0.1

limb = ""

num_arms = 0

#locations on table will be given by function in this file

def log_info(message):
    rospy.loginfo("Robot Interface: {0}".format(message))

def log_err(message):
    rospy.logerr("Robot Interface: {0}".format(message))

def robot_interface():
    rospy.init_node('robot_interface')
    log_info("Initialized Robot Interface")

    environment = rospy.get_param("environment")
    log_info("Desired Environment is: {0}".format(environment))
    if environment == "simulator" or environment == "robot":
        log_info("Beginning to set up robot")
        log_info("Beginning to enable robot")
        global baxter
        baxter = RobotEnable()
        log_info("Enabled Robot")
        
        global num_arms
        num_arms = rospy.get_param("num_arms")
        log_info("Robot is functioning with {0} arms.".format(num_arms))
        global limb
        limb = rospy.get_param("limb")
        log_info("Robot is starting with the {0} arm".format(limb))

        global gripper_left
        global gripper_right
        log_info("Beginning to initialize left gripper")
        gripper_left = Gripper('left')
        log_info("Left Gripper initialized")
        log_info("Beginning to initialize right gripper")
        gripper_right = Gripper('right')
        log_info("Right Gripper initialized")
        log_info("Finished Setting Up Robot.\n\n")
    elif environment == "symbolic":
        log_info("Entering Symbolic Simulator")
    else:
        raise EnvironmentError("Invalid Environment variable")

    log_info("Beginning to start state publisher")
    state_publisher = rospy.Publisher('/state', State, queue_size=10)
    log_info("Successfully started state publisher")

    global right_lock_publisher
    global left_lock_publisher
    log_info("Beginning to initialize locks")
    log_info("Initializing right lock publisher")
    right_lock_publisher = rospy.Publisher('/right_lock', Bool, queue_size=10)
    log_info("Successfully initialized right lock publisher")
    log_info("Initializing left lock publisher")
    left_lock_publisher = rospy.Publisher('/left_lock', Bool, queue_size=10)
    log_info("Initialized left lock publisher")

    # initialize resting poses for both arms
    global rest_pose_right
    global rest_pose_left
    log_info("Initializing rest poses")
    rest_pose_right = Pose()

    rest_pose_right.position = Point(0.576110449966,-0.308593767963,0.246190479074)
    rest_pose_right.orientation = Quaternion(-0.01709843498,0.999460780088,-0.0147234076028,0.0238540113029)

    rest_pose_left = deepcopy(rest_pose_right)
    rest_pose_left.position.y *= -1
    log_info("Rest Poses Initialized")


    # initialize endpoint state so that we will be notified of robot arm position
    log_info("Subscribing to topic /endpoint_state")
    right_limb_subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, respondToEndpointRight)
    log_info("Subscribed to right endpoint state")
    left_limb_subscriber = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, respondToEndpointLeft)
    log_info("Subscribed to left endpoint state")
    rospy.loginfo("Subscribed to topic /endpoint_state")
    
    log_info("Initializing move robot service group")
    log_info("Initializing move robot left service")
    move_robot_left_service = rospy.Service('/move_robot_left', MoveRobot, handle_move_robot_left) # /move_robot
    log_info("Initialized move robot left service")
    log_info("Initializing move robot right service")
    move_robot_right_service = rospy.Service('/move_robot_right', MoveRobot, handle_move_robot_right) # /move_robot
    log_info("Initialized robot right service")

    log_info("Initializing get_state service")
    get_state_service = rospy.Service('/get_state', GetState, handle_get_world_state) # /move_robot
    log_info("Initialized get_state service")

    ns_right = "ExternalTools/right/PositionKinematicsNode/IKService"
    ns_left = "ExternalTools/left/PositionKinematicsNode/IKService"
    try :
        rospy.loginfo("Initializing service proxy for /SolvePositionIK...")
        global iksvc_right
        global iksvc_left
        iksvc_right = rospy.ServiceProxy(ns_right, SolvePositionIK)
        iksvc_left = rospy.ServiceProxy(ns_left, SolvePositionIK)
        rospy.wait_for_service(ns_right, 5.0)
        rospy.wait_for_service(ns_left, 5.0)
        rospy.loginfo("Initialized service proxy for /SolvePositionIK...")
    except rospy.ServiceException, e:
        rospy.logerr("Service Call Failed: {0}".format(e))

    log_info("\nREADY TO MOVE ROBOT\n")

    config = rospy.get_param('configuration')
    log_info("Configuration: {0}".format(config))
    global num_blocks
    num_blocks = rospy.get_param("num_blocks")
    log_info("There are {0} blocks on the table".format(num_blocks))

    global state
    state.gripper_closed = [False,False]
    state.block_in_gripper = [0,0]
    state.stack = range(1, num_blocks+1)
    if config == "stacked_descending" :
        state.stack.reverse()
    state.table = []
    log_info("Initialized State")

    log_info("configuration: %s"%config)
    log_info("num_blocks: %d"%num_blocks)

    broadcast_rate = rospy.Rate(1) # 1 hz
    # HomePose()
    while not rospy.is_shutdown():
        # publish state
        global block_poses
        state.block_poses = block_poses
        state_publisher.publish(state)
        broadcast_rate.sleep()
    
    rospy.spin()

def HomePose() :
    log_info("Going to Home Pose")
    homepose = Pose()
    homepose.position = Point(0.572578886689,0.181184911298,0.146191403844)
    homepose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
    success = MoveToPose(homepose, False, False, False)
    log_info("Got to Home Pose : {0}".format(success))

# updates our known location of hand
# this will update 100 hz
def respondToEndpointRight(EndpointState) :
    global hand_pose_right
    global limb
    hand_pose_right = deepcopy(EndpointState.pose)
    if limb == "right":
        initBlockPositions(EndpointState)

def respondToEndpointLeft(EndpointState) :
    global hand_pose_left
    global limb
    hand_pose_left = deepcopy(EndpointState.pose)
    if limb == "left":
        initBlockPositions(EndpointState)

def initBlockPositions(EndpointState):
    global initial_pose, num_arms,limb,rest_pose_right,rest_pose_left,num_blocks,block_poses,block_size,table_z

    if initial_pose == Pose() :
        log_info("Initializing block positions")

        rospy.sleep(1)

        initial_pose = deepcopy(EndpointState.pose)
        rest_pose_right.position.z = initial_pose.position.z + 2 * block_size
        rest_pose_left.position.z = initial_pose.position.z + 2 * block_size
        # start of robot, save position and set up some positionings
        table_z = initial_pose.position.z - ((num_blocks -1) * block_size)
        for i in range(0,num_blocks) :
            bp = deepcopy(initial_pose)
            bp.position.z -= i * block_size
            block_poses.append(deepcopy(bp))
        if rospy.get_param('configuration') == "stacked_ascending" :
            block_poses.reverse()
        log_info("Initialized block positions")
        # print block_poses

        log_info("Initializing Tower Positions")
        global left_tower, right_tower, mid_tower, left #**HANOI
        left = num_blocks
        left_tower = deepcopy(EndpointState.pose)

        mid_tower = deepcopy(left_tower)
        mid_tower.position.y += block_size * 3
        mid_tower.position.z = table_z - (1 * block_size)

        right_tower = deepcopy(mid_tower)
        right_tower.position.y += block_size * 3
        log_info("Initialized Tower Positions")

        rospy.sleep(1)

def handle_move_robot(req):
    Arm(move_robot, req).start()
    return MoveRobotResponse(True)

def handle_move_robot_left(req):
    Arm(move_robot, req,"left").start()
    return MoveRobotResponse(True)

def handle_move_robot_right(req):
    Arm(move_robot, req,"right").start()
    return MoveRobotResponse(True)

def move_robot(req,reqlimb):
    global right_lock_publisher
    global left_lock_publisher

    idx = -1
    if reqlimb == "left":
        idx = 0
    elif reqlimb == "right":
        idx = 1

    environment = rospy.get_param("environment")
    success = True

    global num_arms
    global block_poses
    global block_size
    global table_z
    global hand_pose_left
    global hand_pose_right
    global gripper_left
    global gripper_right
    global rest_pose_right
    global rest_pose_left
    global state

    global left_tower, mid_tower, right_tower, left, mid, right, active_tower
    
    if req.action == OPEN_GRIPPER_HANOI :

        if environment == "simulator" or environment == "robot":
            log_info("Beginning to open gripper")
            rospy.sleep(GRIPPER_WAIT)
            if reqlimb == "left":
                log_info("Opening Left Gripper")
                gripper_left.open(block=True)
            elif reqlimb == "right":
                log_info("Opening Right Gripper")
                gripper_right.open(block=True)
            rospy.sleep(GRIPPER_WAIT)
            log_info("Opened Gripper")
        elif environment == "symbolic":
            log_info("Pretending to open gripper.")

        if req.target == -2 :
            #towers of hanoi


            if active_tower == 1 :
                left += 1
                left_tower.position.z += block_size
            elif active_tower == 2 :
                mid += 1
                mid_tower.position.z += block_size
            elif active_tower == 3 :
                right += 1
                right_tower.position.z += block_size

    elif req.action == CLOSE_GRIPPER_HANOI :
        if environment == "simulator" or environment == "robot":
            log_info("Beginning to close Gripper")
            # gripper.set_holding_force(5)
            rospy.sleep(GRIPPER_WAIT)

            if reqlimb == "left":
                gripper_left.close(block=True)
            elif reqlimb == "right":
                gripper_right.close(block=True)

            rospy.sleep(GRIPPER_WAIT)
            log_info("Closed Gripper")
        elif environment == "symbolic":
            log_info("Pretending to close gripper")


        if active_tower == 1 :
            left -= 1
            left_tower.position.z -= block_size
        elif active_tower == 2 :
            mid -= 1
            mid_tower.position.z -= block_size
        elif active_tower == 3 :
            right -= 1
            right_tower.position.z -= block_size

        state.gripper_closed[idx] = True
        state.block_in_gripper[idx] = req.target


    elif req.action == MOVE_TO_LEFT_TOWER :
        
        if environment == "simulator" or environment == "robot":
            log_info("Moving to LEFT_TOWER")
            active_tower = 1
            success = MoveToPose(reqlimb, left_tower)
            log_info("Moved To LEFT TOWER: {0}".format(success))
        elif environment == "symbolic":
            log_info("MOVE TO LEFT TOWER")
    elif req.action == MOVE_TO_ABOVE_LEFT_TOWER :
        
        if environment == "simulator" or environment == "robot":
            log_info("Moving to above LEFT_TOWER")
            l = deepcopy(left_tower)
            l.position.z += block_size
            active_tower = 1

            success = MoveToPose(reqlimb, l)
            log_info("Moved To above LEFT TOWER: {0}".format(success))
        elif environment == "symbolic":
            log_info("MOVE TO ABOVE LEFT TOWER")

    elif req.action == MOVE_TO_MID_TOWER :
     
        if environment == "simulator" or environment == "robot":
            log_info("Moving to MID_TOWER")
            active_tower = 2
            success = MoveToPose(reqlimb, mid_tower)
            log_info("Moved To MID TOWER: {0}".format(success))
        elif environment == "symbolic":
            log_info("MOVE TO MID TOWER")


    elif req.action == MOVE_TO_ABOVE_MID_TOWER :
       
        if environment == "simulator" or environment == "robot":
            log_info("Moving to above MID_TOWER")
            l = deepcopy(mid_tower)
            active_tower = 2
            l.position.z += block_size
            success = MoveToPose(reqlimb, l)
            log_info("Moved To ABOVE MID TOWER: {0}".format(success))
        elif environment == "symbolic":
            log_info("MOVE TO ABOVE MID TOWER")


    elif req.action == MOVE_TO_RIGHT_TOWER :
     
        if environment == "simulator" or environment == "robot":
            log_info("Moving to RIGHT TOWER")
            active_tower = 3
            success = MoveToPose(reqlimb, right_tower)
            log_info("Moved To RIGHT TOWER: {0}".format(success))
        elif environment == "symbolic":
            log_info("MOVE TO RIGHT TOWER")

    elif req.action == MOVE_TO_ABOVE_RIGHT_TOWER :
       
        if environment == "simulator" or environment == "robot":
            log_info("Moving to above RIGHT_TOWER")
            l = deepcopy(right_tower)
            active_tower = 3
            l.position.z += block_size
            success = MoveToPose(reqlimb, l)
            log_info("Moved To ABOVE RIGHT TOWER: {0}".format(success))
        elif environment == "symbolic":
            log_info("MOVE TO ABOVE RIGHT TOWER")



    elif req.action == OPEN_GRIPPER :

        # using the target as the destination for this block
        # in practice this means calling MoveRobot -OPEN_GRIPPER with same target as Move_Robot - 
        if environment == "simulator" or environment == "robot":
            log_info("Beginning to open gripper")
            rospy.sleep(GRIPPER_WAIT)

            if reqlimb == "left":
                gripper_left.open(block=True)
            elif reqlimb == "right":
                gripper_right.open(block=True)

            log_info("Opened Gripper")

            if state.block_in_gripper[idx] > 0 :
                # print "Saving block {0} new pose into {1} index".format(state.block_in_gripper,state.block_in_gripper - 1)
                if num_arms == 1:
                    curr_hand_pose = deepcopy(hand_pose_left) if reqlimb == "left" else deepcopy(hand_pose_right)
                    tempidx = state.block_in_gripper[idx]
                    # print "###########################################"
                    # # print tempidx
                    # print len(block_poses)
                    # print block_poses
                    # print block_poses[tempidx - 1]
                    # print "###########################################"
                    block_poses[tempidx - 1] = deepcopy(curr_hand_pose)
                if num_arms == 2:
                    curr_hand_pose = deepcopy(hand_pose_left) if reqlimb == "left" else deepcopy(hand_pose_right)
                    block_poses[(state.block_in_gripper[idx] - 1)] = deepcopy(curr_hand_pose)

            rospy.sleep(GRIPPER_WAIT)

        elif environment == "symbolic":
            log_info("Pretending to open gripper.")

        if state.block_in_gripper[idx] > 0 :
            if req.target == -1 : #putting block on table
                state.table.append(state.block_in_gripper[idx])
                log_info("Deleting {0}".format(state.stack.index(state.block_in_gripper[idx])))
                del state.stack[state.stack.index(state.block_in_gripper[idx])]
            else : #appending to stack
                state.stack.append(state.block_in_gripper[idx])
                del state.table[state.table.index(state.block_in_gripper[idx])]

        state.block_in_gripper[idx] = 0
        state.gripper_closed[idx] = False


    elif req.action == CLOSE_GRIPPER :

        if environment == "simulator" or environment == "robot":
            log_info("Beginning to close Gripper")
            # gripper.set_holding_force(5)
            rospy.sleep(GRIPPER_WAIT)

            if reqlimb == "left": # even block
                gripper_left.close(block=True)
            elif reqlimb == "right": # odd block
                gripper_right.close(block=True)

            rospy.sleep(GRIPPER_WAIT)
            log_info("Closed Gripper")
        elif environment == "symbolic":
            log_info("Pretending to close gripper")

        state.block_in_gripper[idx] = req.target
        state.gripper_closed[idx] = True



    elif req.action == MOVE_TO_BLOCK :

        if environment == "simulator" or environment == "robot":
            log_info("Moving to block {0}".format(req.target))

            if state.block_in_gripper[idx] > 0 or state.gripper_closed[idx]:
                success = False
                log_info("Failed because block in gripper or gripper closed")
            else :
                log_info("Trying to Move to Block {0}".format(req.target))

                success = MoveToPose(reqlimb, block_poses[(req.target-1)])

                log_info("Moved to Block is : {0}".format(success))
        elif environment == "symbolic":
            if state.block_in_gripper[idx] > 0 or state.gripper_closed[idx]:
                success = False
                log_info("Failed because block in gripper or gripper closed")
            else :
                log_info("Pretending to Move to Block {0}".format(req.target))
                log_info("Pretend Moved to Block is : {0}".format(success))



    elif req.action == MOVE_OVER_BLOCK :

        if environment == "simulator" or environment == "robot":
            log_info("Trying to Move OVER Block %d"%req.target)
            temppose = deepcopy(block_poses[(req.target-1)])

            offsetY = 0
            if num_arms == 2:
                offsetY = -.0125 if reqlimb == "left" else 0

            offsetZ = .008
            if req.target in state.stack:
                temppose = deepcopy(initial_pose)
                temppose.position.z = table_z + len(state.stack) * block_size + offsetZ
                temppose.position.y += offsetY
            else:
                temppose.position.z += block_size

            success = MoveToPose(reqlimb, temppose)

            log_info("Moved OVER Block is : {0}".format(success))
        elif environment == "symbolic":
            log_info("Pretending to Move OVER Block %d"%req.target)
        


    elif req.action == MOVE_OVER_TABLE :

        if environment == "simulator" or environment == "robot":
            log_info("Moving over table")

            num_per_row = 4

            delY = ( (req.target-1) % num_per_row + 3 ) * 2 * block_size
            delX = -1 * (int( (req.target-1)/num_per_row ) - 1) * 2 * block_size
            if num_arms == 2:
                if reqlimb == "right":
                    delY = -delY # place to the right

            
            if initial_pose.position.y > 0 and num_arms == 1:
                delY = -delY
            
            p = deepcopy(initial_pose)
            p.position.x += delX
            p.position.y += delY
            p.position.z = table_z

            success = MoveToPose(reqlimb, p)

            log_info("Moved OVER Table : {0}".format(success))
        elif environment == "symbolic":
            log_info("Pretending to move over table")
        


    elif req.action == MOVE_TO_STACK_BOTTOM :

        if environment == "simulator" or environment == "robot":
            log_info("Moving to stack bottom")
            p = deepcopy(initial_pose)
            p.position.z = table_z

            success = MoveToPose(reqlimb, p)

            log_info("Moved To Stack Bottom : {0}".format(success))
        elif environment == "symbolic":
            log_info("Pretending to move to stack bottom")
        


    elif req.action == MOVE_TO_RESTING :
        if environment == "simulator" or environment == "robot":
            log_info("Moving to resting")
            
            p = rest_pose_left if reqlimb == "left" else rest_pose_right
            success = MoveToPose(reqlimb, p, True, False, False)

            log_info("Moved To Resting : {0}".format(success))
        elif environment == "symbolic":
            log_info("Pretending to move to resting")
    else :
        log_err("invalid action")

    if reqlimb == "left":
        log_info("Unlocking left arm")
        left_lock_publisher.publish(False)
        log_info("Unlocked left arm")
    elif reqlimb == "right":
        log_info("Unlocking right arm")
        right_lock_publisher.publish(False)
        log_info("Unlocked right arm")

def handle_get_world_state(req):
    global state
    resp = GetStateResponse()
    resp.gripper_closed = state.gripper_closed
    resp.block_in_gripper = state.block_in_gripper
    resp.stack = state.stack
    resp.table = state.table
    return resp

def MoveToPose (cur_limb, pose, inter1=True, inter2=True, inter3=True) :
    global hand_pose_right
    global hand_pose_left
    global MOVE_WAIT

    new_pose = deepcopy(pose)

    hand_pose = deepcopy(hand_pose_left) if cur_limb == "left" else deepcopy(hand_pose_right)

    if inter1 :
        b1 = MoveToIntermediatePose(cur_limb, hand_pose)
    if inter2 :
        b2 = MoveToIntermediatePose(cur_limb, new_pose)
    if inter2 :
        b3 = MoveToRightAbovePose(cur_limb, new_pose)

    joint_solution = inverse_kinematics(cur_limb, new_pose)
    if joint_solution != [] :
        moveArm(cur_limb,joint_solution)
        rospy.sleep(MOVE_WAIT)
        return True
    else :
        log_err("FAILED MoveToPose")
        return False

def MoveToRightAbovePose(cur_limb, pose) :
    global MOVE_WAIT

    abovepose = deepcopy(pose)
    abovepose.position.z += block_size

    joint_solution = inverse_kinematics(cur_limb, abovepose)
    if joint_solution != [] :
        moveArm(cur_limb, joint_solution)
        rospy.sleep(MOVE_WAIT)
        return True
    else :
        log_err("FAILED MoveToAbovePose")
        return False

def MoveToIntermediatePose(cur_limb, pose) :
    global MOVE_WAIT
    global block_size
    global initial_pose
    global state

    interpose = deepcopy(pose)
    global table_z
    interpose.position.z = table_z + (len(state.stack) + 2) * block_size

    joint_solution = inverse_kinematics(cur_limb, interpose)
    if joint_solution != [] :
        moveArm(cur_limb, joint_solution)
        rospy.sleep(MOVE_WAIT)
        return True
    else :
        log_err("FAILED MoveToIntermediatePose")
        return False

def moveArm (cur_limb, joint_solution) :
    arm = Limb(cur_limb)
    arm.move_to_joint_positions(joint_solution, timeout=7.0)
    rospy.sleep(0.01)

#takes position in base frame of where hand is to go
#calculates ik and moves limb to that location
#returns 1 if successful and 0 if invalid solution
def inverse_kinematics(cur_limb, pose) :
    # given x,y,z will call ik for this position with identity quaternion in base frame
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        cur_limb : PoseStamped(
            header = hdr,
            pose = pose
        ),
    }         
    #getting ik of pose
    
    ikreq.pose_stamp.append(poses[cur_limb])

    try :
        ns = "ExternalTools/"+cur_limb+"/PositionKinematicsNode/IKService"
        rospy.wait_for_service(ns, 5.0)
        if cur_limb == "left":
            resp = iksvc_left(ikreq)
        elif cur_limb == "right":
            resp = iksvc_right(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        log_err("Service call failed: %s" % (e,))
 #       new_pose = deepcopy(pose)
 
        return []

    if (resp.isValid[0]):
        log_info("SUCCESS - Valid Joint Solution Found:")
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print limb_joints
        return limb_joints
    else :
        log_err("Invalid pose")
#        new_pose = deepcopy(pose)

        return []

class Arm(Thread):
    def __init__(self, target, *args):
        Thread.__init__(self)
        self._target = target
        self._args = args
    def run(self):
        self._target(*self._args)

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
