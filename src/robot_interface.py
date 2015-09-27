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

state = State()

hand_pose = Pose() #updated at 100 hz
initial_pose = Pose()

block_size = .04445 #meters, 1.75 inches
num_blocks = 0

stack_x = 0
stack_y = 0
stack_z = 0 #num_blocks * block_size + table_z
table_z = 1 # <---find this

block_poses = [] #positioned at initialization, block 1 at index 0, block 2 at index 1 etc

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
    myx = 0
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
        myx += .01
        broadcast_rate.sleep()

    
    rospy.spin()

#updates our known location of hand
#this will update 100 hz is this inefficient? 
def respondToEndpoint(EndpointState) :
    global hand_pose #can change to deep copy
    hand_pose.position.x = EndpointState.pose.position.x
    hand_pose.position.y = EndpointState.pose.position.y
    hand_pose.position.z = EndpointState.pose.position.z
    hand_pose.orientation.w = EndpointState.pose.orientation.w
    hand_pose.orientation.x = EndpointState.pose.orientation.x
    hand_pose.orientation.y = EndpointState.pose.orientation.y
    hand_pose.orientation.z = EndpointState.pose.orientation.z

    global initial_pose
    if initial_pose == Pose() :
        rospy.loginfo("Initializing block positions")
        initial_pose = hand_pose
        #start of robot, save position and set up some positionings
        global num_blocks
        global block_poses
        global block_size
        for i in range(0,num_blocks) :
            bp = Pose()
            bp.position.x = EndpointState.pose.position.x
            bp.position.y = EndpointState.pose.position.y
            bp.position.z = EndpointState.pose.position.z - i * block_size
            bp.orientation.w = EndpointState.pose.orientation.w
            bp.orientation.x = EndpointState.pose.orientation.x
            bp.orientation.y = EndpointState.pose.orientation.y
            bp.orientation.z = EndpointState.pose.orientation.z
            block_poses.append(bp)
        if rospy.get_param('configuration') == "stack_ascending" :
            block_poses.reverse()
        rospy.loginfo("Initialized block positions")
        print block_poses


def handle_move_robot(req):
    environment = rospy.get_param("environment")
    success = True

    if req.action == OPEN_GRIPPER : #CHRIS  we are using the target as the destination for this block
        #in practice this means calling MoveRobot -OPEN_GRIPPER with same target as Move_Robot - 
        #Move_over_block
        if environment == "simulator" or environment == "robot":
            rospy.loginfo("Beginning to open gripper")
            left_gripper.open()
            rospy.loginfo("Opened Gripper")
        elif environment == "symbolic":
            rospy.loginfo("Pretending to open gripper.")
        
        if req.target == 0 : #putting block on table
            state.table.append(state.block_in_gripper)
            print "Deleting {0}".format(state.stack.index(state.block_in_gripper))
            del state.stack[state.stack.index(state.block_in_gripper)]
        else : #appending to stack
            state.stack.append(state.block_in_gripper)
            del state.table[state.table.index(state.block_in_gripper)]
        
        
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
        print "Moved to block {0}".format(req.target)
        if state.block_in_gripper > 0 or state.gripper_closed:
            success = False

    elif req.action == MOVE_OVER_BLOCK :
        print "Moved over block {0}".format(req.target)
        
    elif req.action == MOVE_OVER_TABLE :
        print "Moved over table"
    elif req.action == MOVE_TO_STACK_BOTTOM :
        print "Moved to stack bottom"
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

def get_current_pose():
    pass

def moveArm (joint_solution, limb) :
    arm = Limb(limb)
    #while not rospy.is_shutdown():
    arm.move_to_joint_positions(joint_solution)
    rospy.sleep(0.01)

#takes position in base frame of where hand is to go
#calculates ik and moves limb to that location
#returns 1 if successful and 0 if invalid solution
def inverse_kinematics(position, orientation) :
    # given x,y,z will call ik for this position with identity quaternion
    #in base frame
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left' : PoseStamped(
            header = hdr,
            pose = Pose (
                position = position,#Point(
	                #x = x,
                    #y = y,
                    #z = z,),
                orientation = orientation,#Quaternion(
                    #x = hand_pose.orientation.x,
                    #y = 0.649877042859,
    	            #z = 0.228353077256,
                    #w = 0.685241671126,),
            ),
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
        return 0
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print limb_joints
	return limb_joints
    else :
        rospy.logerr("Invalid pose")
        return []
    return 1

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
