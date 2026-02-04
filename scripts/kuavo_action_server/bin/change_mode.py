import rospy
import argparse
import sys
from kuavo_msgs.srv import changeArmCtrlMode

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    """调用服务改变手臂控制模式"""
    result = True
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
        )
        change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo("Service call successful")
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
        result = False
    finally:
        return result
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=int, default=2, help="Arm control mode")
    args = parser.parse_args()

    rospy.init_node("change_arm_ctrl_mode_client")
    try:
        res = call_change_arm_ctrl_mode_service(args.mode)
        if res:
            sys.exit(0)
        else:
            sys.exit(1)
    except Exception as e:
        sys.exit(2)