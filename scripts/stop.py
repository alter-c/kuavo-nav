#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty

def main():
    rospy.init_node("navigation_stop")

    pub = rospy.Publisher("/navigation_stop", Empty, queue_size=1)
    rospy.sleep(0.1)

    pub.publish(Empty())
    rospy.loginfo("Send stop signal")

if __name__ == "__main__":
    main()
