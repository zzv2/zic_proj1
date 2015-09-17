#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from zic_proj1.srv import MoveRobot
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

    rospy.loginfo("Beginning to wait for service /move_robot")
    rospy.wait_for_service("/move_robot")
    rospy.loginfo("Finished waiting for service /move_robot")
    try:
        rospy.loginfo("Initializing service proxy for /move_robot...")
        move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
        rospy.loginfo("Initialized service proxy for /move_robot")
    
        # the controller subscribes to the /command topic and listens for commands
        rospy.loginfo("Subscribing to topic /command")
        rospy.Subscriber("command", String, respond_to_command)
        rospy.loginfo("Subscribed to topic /command")

        rospy.spin()

    except rospy.ServiceException, e:
        rospy.logerr("Service Call Failed")





if __name__ == '__main__':
    listener()
        
