import rospy
from dx_nav_common.msg import ObstaclesState

class ObstacleDetector:
    """避障检测模块"""
    def __init__(self, topic="/state_obstacle"):
        self._topic = topic
        self._obstacle = None

        self._obstacle_sub = rospy.Subscriber(
            self._topic, 
            ObstaclesState, 
            self._obstacle_callback
        )

    def _obstacle_callback(self, msg):
        if msg is None:
            rospy.logerr("Received None obstacle message!")
            return
        else:
            # DCCE-ZONE, STOP-ZONE
            if msg.front_info == "NONE":
                self._obstacle = False
            else:
                self._obstacle = True

    def check(self):
        """检查障碍物"""
        return self._obstacle

    def report(self):
        """遇障播报"""
        pass

if __name__ == "__main__":
    rospy.init_node("obstacle_detector_test")
    detector = ObstacleDetector()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        obstacle = detector.check()
        rospy.loginfo(f"Obstacle detected: {obstacle}")
        rate.sleep()
    
    