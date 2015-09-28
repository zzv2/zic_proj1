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

block_size = .04445 #meters, 1.75 inches
num_blocks = 0

table_z = 1 # <---find this

block_poses = [] #positioned at initialization, block 1 at index 0, block 2 at index 1 etc

MOVE_WAIT = .01

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
    
        rospy.loginfo("Beginning to initialize left gripper")
        global left_gripper
        left_gripper = Gripper('left')
        rospy.loginfo("Left Gripper initialized")
    elif environment == "symbolic":
        rospy.loginfo("Entering Symbolic Simulator")
    else:
        raise EnvironmentError("Invalid Environment variable")

    state_publisher = rospy.Publisher('/state', State, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store

    #initialize endpoint state so that we will be notified of robot arm position
    rospy.loginfo("Subscribing to topic /endpoint_state")
    left_limb_subscriber = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, respondToEndpoint)
    rospy.loginfo("Subscribed to topic /endpoint_state")
    
    move_robot_service = rospy.Service('/move_robot', MoveRobot, handle_move_robot) # /move_robot
    get_state_service = rospy.Service('/get_state', GetState, handle_get_world_state) # /move_robot

    ns = "ExternalTools/left/PositionKinematicsNode/IKService"
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

    state.gripper_closed = False
    state.block_in_gripper = 0
    state.stack = range(1, num_blocks+1)
    if config == "stacked_descending" :
        state.stack.reverse()
    state.table = []

    rospy.loginfo("configuration: %s",config)
    rospy.loginfo("num_blocks: %d",num_blocks)

    broadcast_rate = rospy.Rate(1) # 1 hz
    HomePose()
    while not rospy.is_shutdown():
        # publish state
        state_publisher.publish(state)
	    #joint_solution = inverse_kinematics(hand_pose.position.x+.1, hand_pose.position.y, hand_pose.position.z)
	    #print joint_solution
	    #if joint_solution != [] :
	    #moveArm(joint_solution,'left')
        #rospy.loginfo(state)
        #rospy.loginfo("hand pose: ",hand_pose)
        #rospy.loginfo("initial pose: ",initial_pose)
        broadcast_rate.sleep()

    
    rospy.spin()

def HomePose() :
    rospy.loginfo("Going to Home Pose")
    homepose = Pose()
    homepose.position = Point(0.572578886689,0.181184911298,0.146191403844)
    homepose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
    success = MoveToPose(homepose, False)
    rospy.loginfo("Got to Home Pose : %r", success)



#updates our known location of hand
#this will update 100 hz is this inefficient? 
def respondToEndpoint(EndpointState) :
    global hand_pose #can change to deep copy
    hand_pose = deepcopy(EndpointState.pose)
    #hand_pose.position.x = EndpointState.pose.position.x
    #hand_pose.position.y = EndpointState.pose.position.y
    #hand_pose.position.z = EndpointState.pose.position.z
    #hand_pose.orientation.w = EndpointState.pose.orientation.w
    #hand_pose.orientation.x = EndpointState.pose.orientation.x
    #hand_pose.orientation.y = EndpointState.pose.orientation.y
    #hand_pose.orientation.z = EndpointState.pose.orientation.z

    global initial_pose
    if initial_pose == Pose() :
        rospy.loginfo("Initializing block positions")
        initial_pose = hand_pose
        #start of robot, save position and set up some positionings
        global num_blocks
        global block_poses
        global block_size

        global table_z
        table_z = hand_pose.position.z - ((num_blocks -1) * block_size)
        for i in range(0,num_blocks) :
            bp = deepcopy(EndpointState.pose)
            bp.position.z = EndpointState.pose.position.z - i * block_size
            block_poses.append(bp)
        if rospy.get_param('configuration') == "stack_ascending" :
            block_poses.reverse()
        rospy.loginfo("Initialized block positions")
        print block_poses


def handle_move_robot(req):
    environment = rospy.get_param("environment")
    success = True

    global block_poses
    global hand_pose
    global block_size
    global table_z
    if req.action == OPEN_GRIPPER : #CHRIS  we are using the target as the destination for this block
        #in practice this means calling MoveRobot -OPEN_GRIPPER with same target as Move_Robot - 
        #Move_over_block
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Beginning to open gripper")
            left_gripper.open()
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
            left_gripper.close()
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
                print "Moved to Block is : %r" % success


        

    elif req.action == MOVE_OVER_BLOCK :
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Trying to Move OVER Block %d",req.target)
            temppose = deepcopy(block_poses[(req.target-1)])
            temppose.position.z += block_size
            success = MoveToPose(temppose)
            print "Moved OVER Block is : %r" % success
        elif environment == "symbolic":
            rospy.loginfo("Pretending to Move OVER Block %d",req.target)
        
    elif req.action == MOVE_OVER_TABLE :
        if environment == "simulator" or environment == "robot":
            print "Moving over table"
            delY = (1+((req.target - 1) % 5)) * 2 * block_size
            print "DELY : %d", delY
            if initial_pose.position.y > 0 :
                delY = -delY
            delX = (req.target - 1)/5 * 2 * block_size
            
            p = deepcopy(initial_pose)
            p.position.x += delX
            p.position.y += delY
            p.position.z = table_z
            success = MoveToPose(p)
            print "Moved OVER Table : %r" % success
        elif environment == "symbolic":
            print "Pretending over table"
        
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


def handle_get_world_state(req):
    resp = GetStateResponse()
    resp.gripper_closed = state.gripper_closed
    resp.block_in_gripper = state.block_in_gripper
    resp.stack = state.stack
    resp.table = state.table
    return resp

def MoveToPose (pose, intermediate = True) :
    if intermediate :
        b = MoveToIntermediatePose(pose)
        if not b :
            return False

    global MOVE_WAIT
    joint_solution = inverse_kinematics(pose)
    if joint_solution != [] :
        moveArm(joint_solution, 'left')
        rospy.sleep(MOVE_WAIT) #just a made up value atm
        return True
    else :
        rospy.logerr("FAILED MoveToPose")
        return False

def MoveToIntermediatePose(pose) :
    global MOVE_WAIT
    interpose = deepcopy(pose)
    global block_size
    global initial_pose
    interpose.position.z = initial_pose.position.z + 2 * block_size
    joint_solution = inverse_kinematics(interpose)
    if joint_solution != [] :
        moveArm(joint_solution, 'left')
        rospy.sleep(MOVE_WAIT) #just a made up value atm
        return True
    else :
        rospy.logerr("FAILED MoveToIntermediatePose")
        return False

def moveArm (joint_solution, limb) :
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
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left' : PoseStamped(
            header = hdr,
            pose = ourpose
        ),
    }         
    #getting ik of pose
     
    ikreq.pose_stamp.append(poses['left'])

    try :
        ns = "ExternalTools/left/PositionKinematicsNode/IKService"
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
