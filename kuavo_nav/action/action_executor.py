import requests
import random
import rospy

class ActionExecutor:
    def __init__(self, robot: str="kuavo", port: int=8091):
        self.robot = robot
        self.port = port
        self.host_dict = {
            "kuavo": "192.168.26.12",
            # "unitree": "10.0.100.9"
        }
        self.base_url = f"http://{self.host_dict[self.robot]}:{self.port}"

        self.gesture_sign = False

    def execute(self, action: str, mode: int=1):
        info = {
            "robot": self.robot,
            "action": action,
            "mode": mode
        }
        res = requests.post(f"{self.base_url}/api/v1/perform", json=info)
        rospy.loginfo(f"Execute {self.robot} action: {action} in mode {mode}")
        return res.json()
    
    def change_mode(self, mode: int):
        info = {
            "robot": self.robot,
            "action": "",
            "mode": mode
        }
        res = requests.post(f"{self.base_url}/api/v1/mode", json=info)
        rospy.loginfo(f"Change {self.robot} mode to {mode}")
        return res.json()
    
    def gesture(self):
        if self.gesture_sign:
            rospy.logwarn("Gesture already started")
            return
        else:
            self.gesture_sign = True
        gesture_actions = []
        while self.gesture_sign:
            if gesture_actions == []:
                gesture_actions = ["左手扶胸", "点赞", "左手邀请", "右手邀请"]
            action = random.choice(gesture_actions)
            self.execute(action, mode=2)
            gesture_actions.remove(action)
        return
    
    def stop_gesture(self):
        self.gesture_sign = False
        return
    
if __name__ == "__main__":
    executor = ActionExecutor(robot="kuavo")
    result = executor.execute("wave")
    print(result)