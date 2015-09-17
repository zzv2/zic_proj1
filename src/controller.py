#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(command):
    pass

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    rospy.init_node('controller')

    # the controller subscribes to the /command topic and listens for commands
    rospy.Subscriber("command", string, callback)

    rospy.spin()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

        
