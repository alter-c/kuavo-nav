#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import threading
import signal
import sys
import argparse
from std_msgs.msg import String
from flask import Flask, request, jsonify


rospy.init_node('web', anonymous=True)
cmd_pub = rospy.Publisher('/navigation/command', String, queue_size=10)
def get_state() -> dict:
    state_msg = rospy.wait_for_message("/navigation/state", String, timeout=1.0)
    return json.loads(state_msg.data)

# flask app
app = Flask(__name__)

@app.route("/api/navigation/start", methods=["GET"])
def start():
    cid = str(request.args.get("cid"))
    cmd = {
        "command": "navigate",
        "target": cid
    }
    cmd_pub.publish(String(data=json.dumps(cmd)))
    rospy.loginfo(f"Published navigation command: {cmd}")
    nav_state = get_state()
    nav_state["status"] = "starting"
    return jsonify({
        "success": True,
        "message": None,
        "data": nav_state
    })

@app.route("/api/navigation/stop", methods=["GET"])
def stop():
    cmd = {"command": "cancel"}
    cmd_pub.publish(String(data=json.dumps(cmd)))
    rospy.loginfo("Published cancel command")
    nav_state = get_state()
    nav_state.pop("task_id", None)
    nav_state.pop("status", None)
    return jsonify({
        "success": True,
        "message": None,
        "data": nav_state
    })

@app.route("/api/navigation/state", methods=["GET"])
def state():
    task_id = str(request.args.get("task_id"))
    nav_state = get_state()
    if task_id and nav_state["task_id"] == task_id:
        return jsonify({
            "success": True,
            "message": None,
            "data": nav_state
        })
    else:
        return jsonify({
            "success": False,
            "message": "Task not found",
            "data": None
        }), 404

# 仅用于测试位置导航
@app.route("/api/navigation/nav", methods=["GET"])
def nav():
    x = float(request.args.get("x", 0.0))
    y = float(request.args.get("y", 0.0))
    yaw = float(request.args.get("yaw", 0.0))
    cmd = {
        "command": "navigate",
        "target": {"x": x, "y": y, "yaw": yaw}
    }
    cmd_pub.publish(String(data=json.dumps(cmd)))
    rospy.loginfo(f"Published navigation command: {cmd}")
    nav_state = get_state()
    return jsonify({
        "success": True,
        "message": None,
        "data": nav_state
    })

def ros_spin_thread():
    """保持订阅的线程"""
    rospy.spin()

def signal_handler(sig, frame):
    rospy.signal_shutdown("User interrupt")
    sys.exit(0)

def parse_args():
    parser = argparse.ArgumentParser(description="Kuavo Navigation Web Interface")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Web server host")
    parser.add_argument("--port", type=int, default=8080, help="Web server port")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    signal.signal(signal.SIGINT, signal_handler)

    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()

    app.run(host=args.host, port=args.port)
    # curl "http://0.0.0.0:8080/api/navigation/start?cid=target_2"
    # curl "http://0.0.0.0:8080/api/navigation/nav?x=1.0&y=1.0&yaw=1.57"


