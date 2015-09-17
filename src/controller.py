#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from zic_proj1.srv import MoveRobot, GetState
from config import * 

def respond_to_command(command):
    if command == String("scatter"):
        rospy.loginfo("Received Command: scatter")
#        move_robot(OPEN_GRIPPER,0)
    elif command == String("stack_ascending"):
        rospy.loginfo("Received Command: stack_ascending")
    elif command == String("stack_descending"):
        rospy.loginfo("Received Command: stack_ascending")
    else:
        rospy.logerr("Received Malformed Command")

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    rospy.init_node('controller')
    rospy.loginfo("Initialized node Controller")

    # Initialize the /move_robot service from robot_interface
    rospy.loginfo("Beginning to wait for service /move_robot...")
    rospy.wait_for_service("/move_robot")
    rospy.loginfo("Service /get_state now available.")

    # Initialize the /get_state service from robot_interface
    rospy.loginfo("Beginning to wait for service /get_state...")
    rospy.wait_for_service("/get_state")
    rospy.loginfo("Service /get_state now available.")
    try:
        # Initialize the service proxy for the /move_robot server
        rospy.loginfo("Initializing service proxy for /move_robot...")
        move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
        rospy.loginfo("Initialized service proxy for /move_robot")

        # Initialize the service proxy for the /get_state server
        rospy.loginfo("Initializing service proxy for /get_state...")
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
        
