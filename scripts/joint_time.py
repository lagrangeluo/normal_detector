#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def callback_joint_msg(msg):

    new_msg = JointState()

    # 将接收到的 JointState 消息内容复制到新消息中
    new_msg.name = msg.name
    new_msg.position = msg.position
    new_msg.velocity = msg.velocity
    new_msg.effort = msg.effort

    # 将时间戳更新为当前的时间
    new_msg.header.stamp = rospy.Time.now()  # 使用当前时间

    # 发布修改后的消息
    pub.publish(new_msg)
    rospy.loginfo("Published new JointState message with updated timestamp.")

def listener():

    # 创建发布器，用于发布 /joint_states 消息
    global pub
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # 订阅自定义消息
    rospy.Subscriber('/rosbag/joint_states', JointState, callback_joint_msg)
    rospy.spin()
    
  
if __name__ == '__main__':
    rospy.init_node('joint_msg_time_convert', anonymous=True)
    rospy.loginfo("init joint msg convert node")
    listener()
    
