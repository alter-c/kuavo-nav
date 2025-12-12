import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def create_pose_msg(x, y, yaw, frame_id="slam", stamp=None, covariance=None):
    """
    位姿信息封装为PoseWithCovarianceStamped消息
    """
    msg = PoseWithCovarianceStamped()
    
    # 设置 header
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp if stamp else rospy.Time.now()
    
    # 设置位置
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    
    # 设置朝向
    q = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    
    # 设置协方差
    if covariance is None:
        # 默认协方差: 假设精度 x/y ±0.1m, yaw ±0.1rad
        covariance = [0.0] * 36
        covariance[0]  = 0.01  # x
        covariance[7]  = 0.01  # y
        covariance[35] = 0.01  # yaw
    msg.pose.covariance = covariance
    
    return msg