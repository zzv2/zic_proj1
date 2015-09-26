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

    rospy.loginfo("Assembling Initial State from Configuration Variables")
    config = rospy.get_param('configuration')
    n = rospy.get_param("num_blocks")
    r = rospy.get_param("block_radius")
    
    if environment == "simulator" or environment == "robot":
        state.x = [ endpoint_pose['position'].x 
    
    state.gripper_closed = False
    state.block_in_gripper = 0
    if config == "stacked_ascending" :
        state.stack = range(1, n+1)
        x = range(1, n+1)
        y = range(1, n+1)
        z = range(1, n+1)
        i = range(1, n+1)
        j = range(1, n+1)
        k = range(1, n+1)
        w = range(1, n+1)
        for l in range(1, n+1):
            x[l-1] == 0.0
            y[l-1] == 0.0
            z[l-1] == r + 2*(l-1)*r
            i[l-1] == 0.0
            j[l-1] == 0.0
            k[l-1] == 0.0
            w[l-1] == 1.0
    if config == "stacked_descending" :
        state.stack = range(n,0,-1)
        x = range(n, 0, -1)
        y = range(n, 0, -1)
        z = range(n, 0, -1)
        i = range(n, 0, -1)
        j = range(n, 0, -1)
        k = range(n, 0, -1)
        w = range(n, 0, -1)
        for l in range(1, n+1):
            x[l-1] == 0.0
            y[l-1] == 0.0
            z[l-1] == r + 2*(l-1)*r
            i[l-1] == 0.0
            j[l-1] == 0.0
            k[l-1] == 0.0
            w[l-1] == 1.0
    state.table = []

    rospy.loginfo("configuration: %s",config)
    rospy.loginfo("num_blocks: %d",num_blocks)

    broadcast_rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        # publish state
        state_publisher.publish(state)
	inverse_kinematics(.1, .1, .1)
        rospy.loginfo(state)
        broadcast_rate.sleep()

    
    rospy.spin()

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

#takes position in base frame of where hand is to go
#calculates ik and moves limb to that location
#returns 1 if successful and 0 if invalid solution
def inverse_kinematics(x, y, z) :
    # given x,y,z will call ik for this position with identity quaternion
    #in base frame
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left' : PoseStamped(
            header = hdr,
            pose = Pose (
                position = Point(
	            x = x,
                    y = y,
                    z = z,),
                orientation = Quaternion(
                    x = 0,
                    y = 0,
    	            z = 0,
                    w = 1,),
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
        print limb_joints
    else :
        print "Invalid pose"
        return 0
    return 1

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
