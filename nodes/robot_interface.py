#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def robot_interface():
    pub = rospy.Publisher('chatter', String, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store
    /move_robot
    rospy.init_node('robot_interface', anonymous=True)
    rate = rospy.Rate(1) # 1 hz like instructions say

    open_gripper
    close_gripper
    move_to_block #meaning the Fingers are around a block.
    move_over_block
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
