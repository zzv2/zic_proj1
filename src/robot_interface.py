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

def robot_interface():
    rospy.init_node('robot_interface')
    rospy.loginfo("Initialized Robot Interface")

    environment = rospy.get_param("environment")
    rospy.loginfo("Desired Environment is: {0}".format(environment))
    if environment == "simulator" or environment == "robot":
        rospy.loginfo("Beginning to enable robot")
        global baxter
        baxter = RobotEnable()
        rospy.loginfo("Enabled Robot")
        
        global num_arms
        num_arms = rospy.get_param("num_arms")
        global limb
        limb = rospy.get_param("limb")

        global gripper_left
        global gripper_right
        rospy.loginfo("Beginning to initialize left gripper")
        gripper_left = Gripper('left')
        rospy.loginfo("Left Gripper initialized")
        rospy.loginfo("Beginning to initialize right gripper")
        gripper_right = Gripper('right')
        rospy.loginfo("right Gripper initialized")

    elif environment == "symbolic":
        rospy.loginfo("Entering Symbolic Simulator")
    else:
        raise EnvironmentError("Invalid Environment variable")

    state_publisher = rospy.Publisher('/state', State, queue_size=10)

    global right_lock_publisher
    global left_lock_publisher
    right_lock_publisher = rospy.Publisher('/right_lock', Bool, queue_size=10)
    left_lock_publisher = rospy.Publisher('/left_lock', Bool, queue_size=10)

    # initialize resting poses for both arms
    global rest_pose_right
    global rest_pose_left
    rest_pose_right = Pose()

    rest_pose_right.position = Point(0.576110449966,-0.308593767963,0.246190479074)
    rest_pose_right.orientation = Quaternion(-0.01709843498,0.999460780088,-0.0147234076028,0.0238540113029)

    rest_pose_left = deepcopy(rest_pose_right)
    rest_pose_left.position.y *= -1

    # initialize endpoint state so that we will be notified of robot arm position
    rospy.loginfo("Subscribing to topic /endpoint_state")
    right_limb_subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, respondToEndpointRight)
    left_limb_subscriber = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, respondToEndpointLeft)
    rospy.loginfo("Subscribed to topic /endpoint_state")
    
    move_robot_left_service = rospy.Service('/move_robot_left', MoveRobot, handle_move_robot_left) # /move_robot
    move_robot_right_service = rospy.Service('/move_robot_right', MoveRobot, handle_move_robot_right) # /move_robot
    get_state_service = rospy.Service('/get_state', GetState, handle_get_world_state) # /move_robot

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

    print "Ready to move robot."

    config = rospy.get_param('configuration')
    global num_blocks
    num_blocks = rospy.get_param("num_blocks")

    global state
    state.gripper_closed = [False,False]
    state.block_in_gripper = [0,0]
    state.stack = range(1, num_blocks+1)
    if config == "stacked_descending" :
        state.stack.reverse()
    state.table = []

    rospy.loginfo("configuration: %s",config)
    rospy.loginfo("num_blocks: %d",num_blocks)

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
    rospy.loginfo("Going to Home Pose")
    homepose = Pose()
    homepose.position = Point(0.572578886689,0.181184911298,0.146191403844)
    homepose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
    success = MoveToPose(homepose, False, False, False)
    rospy.loginfo("Got to Home Pose : %r", success)

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
        rospy.loginfo("Initializing block positions")

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
        rospy.loginfo("Initialized block positions")
        # print block_poses

    global left_tower, right_tower, mid_tower, left #**HANOI
    left = num_blocks
    left_tower = deepcopy(EndpointState.pose)

    mid_tower = deepcopy(left_tower)
    mid_tower.position.y += block_size * 3
    mid_tower.position.z = table_z - (1 * block_size)

    right_tower = deepcopy(mid_tower)
    right_tower.position.y += block_size * 3


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


    # print "####################################################"
    # print reqlimb
    # print "####################################################"
    environment = rospy.get_param("environment")
    success = True
    # block_in_gripper = state.block_in_gripper_left if limb == "left" else state.block_in_gripper_right
    # gripper_closed = state.gripper_closed_left if limb == "left" else state.gripper_closed_right

    # print "###############################################"
    # print "limb: {0}, block_in_gripper: {1}, gripper_closed: {2}".format(reqlimb,block_in_gripper,gripper_closed)
    # print "###############################################"


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

    global left_tower, mid_tower, right_tower, left, mid, right, active_tower
    
    if req.action == OPEN_GRIPPER_HANOI :

        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Beginning to open gripper")
            rospy.sleep(GRIPPER_WAIT)
            gripper.open(block=True)
            rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Opened Gripper")
        elif environment == "symbolic":
            rospy.loginfo("Pretending to open gripper.")

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
            rospy.loginfo("Beginning to close Gripper")
            # gripper.set_holding_force(5)
            rospy.sleep(GRIPPER_WAIT)
            gripper.close(block=True)
            rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Closed Gripper")
        elif environment == "symbolic":
            rospy.loginfo("Pretending to close gripper")


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


    elif req.action == MOVE_TO_LEFT_TOWER : #***********************HANOI 286 add special code to open
        
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to LEFT_TOWER")
            active_tower = 1
            success = MoveToPose(left_tower)
            print "Moved To LEFT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO LEFT TOWER")
    elif req.action == MOVE_TO_ABOVE_LEFT_TOWER :
        
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to above LEFT_TOWER")
            l = deepcopy(left_tower)
            l.position.z += block_size
            active_tower = 1

            success = MoveToPose(l)
            print "Moved To above LEFT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO ABOVE LEFT TOWER")

    elif req.action == MOVE_TO_MID_TOWER : #***********************HANOI 286 add special code to open
     
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to MID_TOWER")
            active_tower = 2
            success = MoveToPose(mid_tower)
            print "Moved To MID TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO MID TOWER")


    elif req.action == MOVE_TO_ABOVE_MID_TOWER :
       
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to above MID_TOWER")
            l = deepcopy(mid_tower)
            active_tower = 2
            l.position.z += block_size
            success = MoveToPose(l)
            print "Moved To ABOVE MID TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO ABOVE MID TOWER")


    elif req.action == MOVE_TO_RIGHT_TOWER : #***********************HANOI 286 add special code to open
     
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to RIGHT TOWER")
            active_tower = 3
            success = MoveToPose(right_tower)
            print "Moved To RIGHT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO RIGHT TOWER")

    elif req.action == MOVE_TO_ABOVE_RIGHT_TOWER :
       
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to above RIGHT_TOWER")
            l = deepcopy(right_tower)
            active_tower = 3
            l.position.z += block_size
            success = MoveToPose(l)
            print "Moved To ABOVE RIGHT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO ABOVE RIGHT TOWER")



    elif req.action == OPEN_GRIPPER :

        # using the target as the destination for this block
        # in practice this means calling MoveRobot -OPEN_GRIPPER with same target as Move_Robot - 
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Beginning to open gripper")
            rospy.sleep(GRIPPER_WAIT)

            if reqlimb == "left":
                gripper_left.open(block=True)
            elif reqlimb == "right":
                gripper_right.open(block=True)

            rospy.loginfo("Opened Gripper")

            # print "##################################"
            # print block_poses
            # print "##################################"

            if state.block_in_gripper[idx] > 0 :
                # print "Saving block {0} new pose into {1} index".format(state.block_in_gripper,state.block_in_gripper - 1)
                if num_arms == 1:
                    curr_hand_pose = deepcopy(hand_pose_left) if reqlimb == "left" else deepcopy(hand_pose_right)
                    block_poses[(block_in_gripper - 1)] = deepcopy(curr_hand_pose)
                if num_arms == 2:
                    curr_hand_pose = deepcopy(hand_pose_left) if reqlimb == "left" else deepcopy(hand_pose_right)
                    block_poses[(block_in_gripper - 1)] = deepcopy(curr_hand_pose)

            rospy.sleep(GRIPPER_WAIT)

            # move the arms out of the way after they put down the block
            if num_arms == 2:
                if reqlimb == "left":
                    success = MoveToPose("left", rest_pose_left, True, False, False)
                elif reqlimb == "right":
                    success = MoveToPose("right", rest_pose_right, True, False, False)

        elif environment == "symbolic":
            rospy.loginfo("Pretending to open gripper.")

        if state.block_in_gripper[idx] > 0 :
            if req.target == -1 : #putting block on table
                state.table.append(block_in_gripper)
                print "Deleting {0}".format(state.stack.index(block_in_gripper))
                del state.stack[state.stack.index(block_in_gripper)]
            else : #appending to stack
                state.stack.append(block_in_gripper)
                del state.table[state.table.index(block_in_gripper)]

        state.block_in_gripper[idx] = 0
        state.gripper_closed[idx] = False


    elif req.action == CLOSE_GRIPPER :

        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Beginning to close Gripper")
            # gripper.set_holding_force(5)
            rospy.sleep(GRIPPER_WAIT)

            if reqlimb == "left": # even block
                gripper_left.close(block=True)
            elif reqlimb == "right": # odd block
                gripper_right.close(block=True)

            rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Closed Gripper")
        elif environment == "symbolic":
            rospy.loginfo("Pretending to close gripper")

        state.block_in_gripper[idx] = req.target
        state.gripper_closed[idx] = True



    elif req.action == MOVE_TO_BLOCK :

        if environment == "simulator" or environment == "robot":
            print "Moving to block {0}".format(req.target)

            # print "###############################################"
            # print "limb: {0}, block_in_gripper: {1}, gripper_closed: {2}".format(reqlimb,block_in_gripper,gripper_closed)
            # print "state.block_in_gripper_left: {0}, state.block_in_gripper_right: {1}".format(state.block_in_gripper_left,state.block_in_gripper_right)
            # print "state.gripper_closed_left: {0}, state.gripper_closed_right: {1}".format(state.gripper_closed_left,state.gripper_closed_right)
            # print "###############################################"

            if state.block_in_gripper[idx] > 0 or state.gripper_closed[idx]:
                success = False
                rospy.loginfo("Failed because block in gripper or gripper closed")
            else :
                rospy.loginfo("Trying to Move to Block %d",req.target)

                success = MoveToPose(reqlimb, block_poses[(req.target-1)])

                print "Moved to Block is : %r" % success
        elif environment == "symbolic":
            if state.block_in_gripper[idx] > 0 or state.gripper_closed[idx]:
                success = False
                rospy.loginfo("Failed because block in gripper or gripper closed")
            else :
                rospy.loginfo("Pretending to Move to Block %d",req.target)
                print "Pretend Moved to Block is : %r" % success



    elif req.action == MOVE_OVER_BLOCK :

        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Trying to Move OVER Block %d",req.target)
            temppose = deepcopy(block_poses[(req.target-1)])

            offset = .008
            if req.target in state.stack:
                temppose = deepcopy(initial_pose)
                temppose.position.z = table_z + len(state.stack) * block_size + offset
            else:
                temppose.position.z += block_size

            success = MoveToPose(reqlimb, temppose)

            print "Moved OVER Block is : %r" % success
        elif environment == "symbolic":
            rospy.loginfo("Pretending to Move OVER Block %d",req.target)
        


    elif req.action == MOVE_OVER_TABLE :

        if environment == "simulator" or environment == "robot":
            print "Moving over table"

            num_per_row = 4

            delY = ( (req.target-1) % num_per_row + 2 ) * 2 * block_size
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

            print "Moved OVER Table : %r" % success
        elif environment == "symbolic":
            print "Pretending to move over table"
        


    elif req.action == MOVE_TO_STACK_BOTTOM :

        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to stack bottom")
            p = deepcopy(initial_pose)
            p.position.z = table_z

            success = MoveToPose(reqlimb, p)

            print "Moved To Stack Bottom : %r" % success
        elif environment == "symbolic":
            rospy.loginfo("Pretending to move to stack bottom")
        


    elif req.action == WAIT_FOR_DONE :
        rospy.loginfo("Waiting for arm to finish.")
    else :
        print "invalid action"

    # if reqlimb == "left":
    #     left_lock_publisher.publish(False)
    # elif reqlimb == "right":
    #     right_lock_publisher.publish(False)

def pick_and_place(source, destination):
    """ Routine is a stub, implemented for part 2(b) """
    pass


def handle_get_world_state(req):
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
        rospy.logerr("FAILED MoveToPose")
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
        rospy.logerr("FAILED MoveToAbovePose")
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
        rospy.sleep(MOVE_WAIT) #just a made up value atm
        return True
    else :
        rospy.logerr("FAILED MoveToIntermediatePose")
        return False

def moveArm (cur_limb, joint_solution) :
    arm = Limb(cur_limb)
    arm.move_to_joint_positions(joint_solution)
    rospy.sleep(0.01)

#takes position in base frame of where hand is to go
#calculates ik and moves limb to that location
#returns 1 if successful and 0 if invalid solution
def inverse_kinematics(cur_limb, pose) :
    # given x,y,z will call ik for this position with identity quaternion
    # in base frame
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
        rospy.logerr("Service call failed: %s" % (e,))
        return []
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print limb_joints
        return limb_joints
    else :
        rospy.logerr("Invalid pose")
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
