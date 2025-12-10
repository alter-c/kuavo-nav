#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import threading
import signal
import sys
from std_msgs.msg import String, Bool
from flask import Flask, request, jsonify

rospy.init_node('web', anonymous=True)
cmd_pub = rospy.Publisher('/navigation/command', String, queue_size=10)
app = Flask(__name__)

@app.route("/api/navigation/start", methods=["GET"])
def start():
    x = float(request.args.get("x", 0.0))
    y = float(request.args.get("y", 0.0))
    yaw = float(request.args.get("yaw", 0.0))
    cmd = {
        "command": "navigate",
        "target": {
            "x": x,
            "y": y,
            "yaw": yaw
        }
    }
    cmd_pub.publish(String(data=json.dumps(cmd)))
    rospy.loginfo(f"Published navigation command: {cmd}")
    return

@app.route("/api/navigation/stop", methods=["GET"])
def stop():
    cmd = {"command": "cancel"}
    cmd_pub.publish(String(data=json.dumps(cmd)))
    rospy.loginfo("Published cancel command")
    return 

@app.route("/api/navigation/state", methods=["GET"])
def state():
    nav_state = rospy.wait_for_message("/navigation/state", Bool, timeout=1.0)
    rospy.loginfo(f"Received state: {nav_state.data}")
    return jsonify({
        "state": nav_state.data
        })

def ros_spin_thread():
    """保持订阅的线程"""
    rospy.spin()

def signal_handler(sig, frame):
    rospy.signal_shutdown("User interrupt")
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()

    app.run(host='0.0.0.0', port=8080)
    # curl "http://0.0.0.0:8080/api/navigation/start?x=1.0&y=1.0&yaw=1.57"


