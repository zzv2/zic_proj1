#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def respond_to_command(command):
    if command == "scatter":
        rospy.loginfo("Received Command: scatter")
    elif command == "stack_ascending":
        rospy.loginfo("Received Command: stack_ascending")
    elif command == "stack_descending":
        rospy.loginfo("Received Command: stack_ascending")
    else:
        rospy.logerr("Received Malformed Command")

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    rospy.init_node('controller')

    # the controller subscribes to the /command topic and listens for commands
    rospy.Subscriber("command", String, respond_to_command)

    rospy.spin()



if __name__ == '__main__':
    listener()
        
