import rospy
import threading
from std_msgs.msg import String, Bool

from .navigation.core import NavigationCore

class StatePublisher:
    def __init__(
        self,
        navigator: NavigationCore=None,
        rate_hz: int=10
    ):
        if navigator is None:
            raise ValueError("NavigationCore is not initialized")
        
        self._navigator = navigator

        self.rate = rospy.Rate(rate_hz)
        self.state_pub = rospy.Publisher("/navigation/state", Bool, queue_size=10)
        self.state_thread = None
        
    def start(self):
        self.state_thread = threading.Thread(target=self._state_pub_loop, daemon=True)
        self.state_thread.start()
        rospy.loginfo("Navigation info publisher started!")

    def _state_pub_loop(self):
        while not rospy.is_shutdown():
            msg = Bool()
            msg.data = self._navigator.is_running
            self.state_pub.publish(msg)
            self.rate.sleep()
    