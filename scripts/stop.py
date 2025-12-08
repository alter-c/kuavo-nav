#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty

def main():
    rospy.init_node("navigation_stop_sender")

    pub = rospy.Publisher("/navigation_stop", Empty, queue_size=1)

    # 等 publisher 建立
    rospy.sleep(0.2)

    rospy.loginfo("Sending navigation stop signal")
    pub.publish(Empty())

    rospy.loginfo("Stop signal sent")

if __name__ == "__main__":
    main()
