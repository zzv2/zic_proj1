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

state = State()

hand_pose = Pose() #updated at 100 hz
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

MOVE_WAIT = 0.1
GRIPPER_WAIT = 0.05

limb = ""

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
        
        global limb
        limb = rospy.get_param("limb")
        global gripper
        if limb == "left":
            rospy.loginfo("Beginning to initialize left gripper")
            gripper = Gripper(limb)
            rospy.loginfo("Left Gripper initialized")
        elif limb == "right":
            rospy.loginfo("Beginning to initialize right gripper")
            gripper = Gripper(limb)
            rospy.loginfo("right Gripper initialized")
    elif environment == "symbolic":
        rospy.loginfo("Entering Symbolic Simulator")
    else:
        raise EnvironmentError("Invalid Environment variable")

    state_publisher = rospy.Publisher('/state', State, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store

    #initialize endpoint state so that we will be notified of robot arm position
    rospy.loginfo("Subscribing to topic /endpoint_state")
    limb_subscriber = rospy.Subscriber("/robot/limb/"+limb+"/endpoint_state", EndpointState, respondToEndpoint)
    rospy.loginfo("Subscribed to topic /endpoint_state")
    
    move_robot_service = rospy.Service('/move_robot', MoveRobot, handle_move_robot) # /move_robot
    get_state_service = rospy.Service('/get_state', GetState, handle_get_world_state) # /move_robot

    ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
    try :
        rospy.loginfo("Initializing service proxy for /SolvePositionIK...")
        global iksvc
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        rospy.loginfo("Initialized service proxy for /SolvePositionIK...")
    except rospy.ServiceException, e:
        rospy.logerr("Service Call Failed: {0}".format(e))
        

    print "Ready to move robot."

    config = rospy.get_param('configuration')
    global num_blocks
    num_blocks = rospy.get_param("num_blocks")

    global state
    global block_poses
    state.gripper_closed = False
    state.block_in_gripper = 0
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
def respondToEndpoint(EndpointState) :
    global hand_pose
    hand_pose = deepcopy(EndpointState.pose)
    initBlockPositions(EndpointState)

def initBlockPositions(EndpointState):
    global initial_pose
    if initial_pose == Pose() :
        rospy.loginfo("Initializing block positions")
        initial_pose = hand_pose
        # start of robot, save position and set up some positionings
        global num_blocks
        global block_poses
        global block_size

        global table_z
        table_z = hand_pose.position.z - ((num_blocks -1) * block_size)
        for i in range(0,num_blocks) :
            bp = deepcopy(EndpointState.pose)
            bp.position.z = EndpointState.pose.position.z - i * block_size
            block_poses.append(bp)
        if rospy.get_param('configuration') == "stacked_ascending" :
            block_poses.reverse()
        rospy.loginfo("Initialized block positions")
        # print block_poses

        global left_tower, right_tower, mid_tower, left #**HANOI
        left = num_blocks
        left_tower = deepcopy(EndpointState.pose)

        mid_tower = deepcopy(left_tower)
        mid_tower.position.y += block_size * 3
        mid_tower.position.z = table_z - (2 * block_size)

        right_tower = deepcopy(mid_tower)
        right_tower.position.y += block_size * 3


def handle_move_robot(req):
    environment = rospy.get_param("environment")
    success = True

    global block_poses
    global hand_pose
    global block_size
    global table_z

    global left_tower, mid_tower, right_tower, left, mid, right
    
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

            active_tower = ClosestToTower(hand_pose.position)

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

        active_tower = ClosestToTower(hand_pose.position)

        if active_tower == 1 :
            left -= 1
            left_tower.position.z -= block_size
        elif active_tower == 2 :
            mid -= 1
            mid_tower.position.z -= block_size
        elif active_tower == 3 :
            right -= 1
            right_tower.position.z -= block_size

        state.gripper_closed = True
        state.block_in_gripper = req.target


    elif req.action == MOVE_TO_LEFT_TOWER : #***********************HANOI 286 add special code to open
        
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to LEFT_TOWER")
            success = MoveToPose(left_tower)
            print "Moved To LEFT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO LEFT TOWER")
    elif req.action == MOVE_TO_ABOVE_LEFT_TOWER :
        
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to above LEFT_TOWER")
            l = deepcopy(left_tower)
            l.position.z += block_size
            success = MoveToPose(l)
            print "Moved To above LEFT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO ABOVE LEFT TOWER")

    elif req.action == MOVE_TO_MID_TOWER : #***********************HANOI 286 add special code to open
     
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to MID_TOWER")
            success = MoveToPose(mid_tower)
            print "Moved To MID TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO MID TOWER")


    elif req.action == MOVE_TO_ABOVE_MID_TOWER :
       
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to above MID_TOWER")
            l = deepcopy(mid_tower)
            l.position.z += block_size
            success = MoveToPose(l)
            print "Moved To ABOVE MID TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO ABOVE MID TOWER")


    elif req.action == MOVE_TO_RIGHT_TOWER : #***********************HANOI 286 add special code to open
     
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to RIGHT TOWER")
            success = MoveToPose(right_tower)
            print "Moved To RIGHT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO RIGHT TOWER")

    elif req.action == MOVE_TO_ABOVE_RIGHT_TOWER :
       
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to above RIGHT_TOWER")
            l = deepcopy(right_tower)
            l.position.z += block_size
            success = MoveToPose(l)
            print "Moved To ABOVE RIGHT TOWER: %r" % success
        elif environment == "symbolic":
            rospy.loginfo("MOVE TO ABOVE RIGHT TOWER")


    elif req.action == OPEN_GRIPPER : # using the target as the destination for this block
        # in practice this means calling MoveRobot -OPEN_GRIPPER with same target as Move_Robot - 
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Beginning to open gripper")
            rospy.sleep(GRIPPER_WAIT)
            gripper.open(block=True)
            rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Opened Gripper")
        elif environment == "symbolic":
            rospy.loginfo("Pretending to open gripper.")

        if state.block_in_gripper > 0 :
            if req.target == -1 : #putting block on table
                state.table.append(state.block_in_gripper)
                print "Deleting {0}".format(state.stack.index(state.block_in_gripper))
                del state.stack[state.stack.index(state.block_in_gripper)]
            else : #appending to stack
                state.stack.append(state.block_in_gripper)
                del state.table[state.table.index(state.block_in_gripper)]
            
            block_poses[(state.block_in_gripper - 1)] = deepcopy(hand_pose)

        state.block_in_gripper = 0
        state.gripper_closed = False

    elif req.action == CLOSE_GRIPPER :

        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Beginning to close Gripper")
            # gripper.set_holding_force(5)
            rospy.sleep(GRIPPER_WAIT)
            gripper.close(block=True)
            rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Closed Gripper")
        elif environment == "symbolic":
            rospy.loginfo("Pretending to close gripper")

        state.gripper_closed = True
        state.block_in_gripper = req.target

    elif req.action == MOVE_TO_BLOCK :
        if environment == "simulator" or environment == "robot":
            print "Moving to block {0}".format(req.target)
            if state.block_in_gripper > 0 or state.gripper_closed:
                success = False
                rospy.loginfo("Failed because block in gripper or gripper closed")
            else :
                rospy.loginfo("Trying to Move to Block %d",req.target)
                success = MoveToPose(block_poses[(req.target-1)])
                print "Moved to Block is : %r" % success
        elif environment == "symbolic":
            if state.block_in_gripper > 0 or state.gripper_closed:
                success = False
                rospy.loginfo("Failed because block in gripper or gripper closed")
            else :
                rospy.loginfo("Pretending to Move to Block %d",req.target)
                print "Pretend Moved to Block is : %r" % success


        

    elif req.action == MOVE_OVER_BLOCK :
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Trying to Move OVER Block %d",req.target)
            temppose = deepcopy(block_poses[(req.target-1)])
            # temppose.position.z += block_size + .001
            temppose.position.z = table_z + len(state.stack) * block_size + .008
            success = MoveToPose(temppose)
            print "Moved OVER Block is : %r" % success
        elif environment == "symbolic":
            rospy.loginfo("Pretending to Move OVER Block %d",req.target)
        
    elif req.action == MOVE_OVER_TABLE :
        if environment == "simulator" or environment == "robot":
            print "Moving over table"

            num_per_row = 4
            delY = (2 + (req.target-1) % num_per_row) * 2 * block_size
            delX = -1 * (int( (req.target-1)/num_per_row ) + 0) * 2 * block_size
            
            # print "DELY : %d", delY
            if initial_pose.position.y > 0 :
                delY = -delY
            
            p = deepcopy(initial_pose)
            p.position.x += delX
            p.position.y += delY
            p.position.z = table_z
            rospy.loginfo(p.position)
            success = MoveToPose(p)
            print "Moved OVER Table : %r" % success
        elif environment == "symbolic":
            print "Pretending to move over table"
        
    elif req.action == MOVE_TO_STACK_BOTTOM :
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Moving to stack bottom")
            p = deepcopy(initial_pose)
            p.position.z = table_z
            success = MoveToPose(p)
            print "Moved To Stack Bottom : %r" % success
        elif environment == "symbolic":
            rospy.loginfo("Pretending to move to stack bottom")
    else :
        print "invalid action"

    return MoveRobotResponse(success)

def pick_and_place(source, destination):
    """ Routine is a stub, implemented for part 2(b) """
    pass

def ClosestToTower(position) : #*******************HANOI 280
    global left_tower, mid_tower, right_tower
    dl = PointDistance(position,left_tower.position)
    dm = PointDistance(position, mid_tower.position)
    dr = PointDistance(position, right_tower.position)

    m = min(min (dl,dm),dr)
    if dl == m :
        return 1
    elif dm == m :
        return 2
    else :
        return 3

def PointDistance(p1, p2) :
    return (((p1.x - p2.x) ** 2) + ((p1.y - p2.y) ** 2) + ((p1.z - p2.z) ** 2)) ** .5

def handle_get_world_state(req):
    resp = GetStateResponse()
    resp.gripper_closed = state.gripper_closed
    resp.block_in_gripper = state.block_in_gripper
    resp.stack = state.stack
    resp.table = state.table
    return resp

def MoveToPose (pose, inter1=True, inter2=True, inter3=True) :
    global hand_pose
    global MOVE_WAIT

    if inter1 :
        b1 = MoveToIntermediatePose(hand_pose)
    if inter2 :
        b2 = MoveToIntermediatePose(pose)
    if inter2 :
        b3 = MoveToRightAbovePose(pose)

    joint_solution = inverse_kinematics(pose)
    if joint_solution != [] :
        moveArm(joint_solution)
        rospy.sleep(MOVE_WAIT)
        return True
    else :
        rospy.logerr("FAILED MoveToPose")
        return False

def MoveToRightAbovePose(pose) :
    global MOVE_WAIT

    abovepose = deepcopy(pose)
    abovepose.position.z += block_size

    joint_solution = inverse_kinematics(abovepose)
    if joint_solution != [] :
        moveArm(joint_solution)
        rospy.sleep(MOVE_WAIT)
        return True
    else :
        rospy.logerr("FAILED MoveToAbovePose")
        return False

def MoveToIntermediatePose(pose) :
    global MOVE_WAIT
    global block_size
    global initial_pose
    global state
    global limb

    interpose = deepcopy(pose)
    interpose.position.z = (len(state.stack) + 1) * block_size

    joint_solution = inverse_kinematics(interpose)
    if joint_solution != [] :
        moveArm(joint_solution)
        rospy.sleep(MOVE_WAIT) #just a made up value atm
        return True
    else :
        rospy.logerr("FAILED MoveToIntermediatePose")
        return False

def moveArm (joint_solution) :
    global limb
    arm = Limb(limb)
    #while not rospy.is_shutdown():
    arm.move_to_joint_positions(joint_solution)
    rospy.sleep(0.01)

#takes position in base frame of where hand is to go
#calculates ik and moves limb to that location
#returns 1 if successful and 0 if invalid solution
def inverse_kinematics(ourpose) :
    # given x,y,z will call ik for this position with identity quaternion
    #in base frame
    global limb
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        limb : PoseStamped(
            header = hdr,
            pose = ourpose
        ),
    }         
    #getting ik of pose
     
    ikreq.pose_stamp.append(poses[limb])

    try :
        ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
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

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
