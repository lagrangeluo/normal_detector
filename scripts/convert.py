#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from frcobot_hw.msg import status  # 导入自定义消息类型
from std_msgs.msg import Header

scale = 3.1415/180

def callback_frrobot_msg(data):

    global scale
    """
    处理自定义消息的回调函数
    """
    # 解析自定义消息（根据 CustomMsg 的具体字段来解析）
    joint_positions = data.cur_joints_pose
    joint_msg = []
    
    for joint in joint_positions:
        convert = joint*scale
        joint_msg.append(convert)
    
    tcp_pose = data.cur_tcp_pose

    # 创建 JointState 消息并发布
    joint_state_msg = JointState()
    
    # 设置时间戳
    joint_state_msg.header = Header()
    joint_state_msg.header.stamp = rospy.Time.now()

    # 填充 JointState 消息
    joint_state_msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']  # 填写你的关节名称
    joint_state_msg.position = joint_msg
    joint_state_msg.velocity = []
    joint_state_msg.effort = []

    # 发布 JointState 消息
    pub.publish(joint_state_msg)

def listener():
    """
    订阅自定义消息并发布 JointState 消息的函数
    """
    rospy.init_node('frrobot_msg_to_joint_state', anonymous=True)
    rospy.loginfo("init frrobot msg convert node")

    # 创建发布器，用于发布 /joint_states 消息
    global pub
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # 订阅自定义消息
    rospy.Subscriber('/frcobot_status', status, callback_frrobot_msg)

    rospy.spin()

if __name__ == '__main__':
    listener()

