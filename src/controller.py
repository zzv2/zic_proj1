#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from zic_proj1.srv import MoveRobot
from config import * 

def respond_to_command(command):
    if command == String("scatter"):
        rospy.loginfo("Received Command: scatter")
        move_robot(OPEN_GRIPPER,0)
    elif command == String("stack_ascending"):
        rospy.loginfo("Received Command: stack_ascending")
    elif command == String("stack_descending"):
        rospy.loginfo("Received Command: stack_ascending")
    else:
        rospy.logerr("Received Malformed Command")

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    rospy.init_node('controller')

    rospy.wait_for_service("/move_robot")
    try:
        move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
    
        # the controller subscribes to the /command topic and listens for commands
        rospy.Subscriber("command", String, respond_to_command)

    except rospy.ServiceException, e:
        rospy.logerr("Service Call Failed")


    rospy.spin()



if __name__ == '__main__':
    listener()
        
