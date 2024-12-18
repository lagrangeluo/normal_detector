#!/usr/bin/env python3

import os
import time
import rospy
from sensor_msgs.msg import JointState
from frcobot_hw.msg import status
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
import argparse
import json

class data_saver():
    
    def __init__(self,args):
        # global pub
        # pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.get_arm_state_flag = False
        self.arm_tcp_list = []
        self.arm_joint_list = []
        # json数据写入缓冲区
        self.json_data_buffer = []

        # 构建json文件保存路径,构建上一级目录的路径
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        # 构建目标文件夹路径
        data_folder = os.path.join(parent_dir, 'data')
        # 如果 data 文件夹不存在，则创建它
        os.makedirs(data_folder, exist_ok=True)

        # 构建 JSON 文件的完整路径
        self.json_file_path = os.path.join(data_folder, args.data_name+'.json')
        rospy.loginfo(f"json file path: {self.json_file_path}")
        # 打开文件并清空内容
        with open(self.json_file_path, 'w') as json_file:
            pass  # 不写入任何内容，直接清空文件
        
        # 时间控制
        self.last_time = rospy.Time(0)  # 初始化为 0，表示尚未处理过消息
        self.time_interval = rospy.Duration(0.02)  # 20ms（50Hz）
        
        # 订阅自定义消息
        rospy.Subscriber('/frcobot_status', status, self.arm_state_callback)
        rospy.Subscriber('/normals', MarkerArray, self.callback_normal_msg)
        
    def callback_normal_msg(self,data):
        
        current_time = rospy.Time.now()
        # 如果当前时间与上次时间间隔大于设定值，则执行回调
        if current_time - self.last_time > self.time_interval:
            self.last_time = current_time  # 更新最后执行时间
        else:
            return
        
        if len(data.markers[0].points) == 0:
            rospy.logwarn("Marker or points is empty")
            return
        
        rospy.loginfo("Received message: ")
        #换源normal法向量各轴数值
        point_x_start = data.markers[0].points[0].x
        point_y_start = data.markers[0].points[0].y
        point_z_start = data.markers[0].points[0].z
        
        point_x_end = data.markers[0].points[1].x
        point_y_end = data.markers[0].points[1].y
        point_z_end = data.markers[0].points[1].z

        normal_x = (point_x_end - point_x_start)*10
        normal_y = (point_y_end - point_y_start)*10
        normal_z = (point_z_end - point_z_start)*10

        rospy.loginfo(f"normal_x: {normal_x} normal_y: {normal_y} normal_z: {normal_z}")
        
        #获取关节角度
        if not self.arm_tcp_list or not self.arm_joint_list:
            rospy.logwarn("arm tcp or joint list is empty")
            return
        
        # 创建字典和json对象
        json_dic = {
            "normal": [
                normal_x,normal_y,normal_z
            ],
            "tcp": [
                self.arm_tcp_list[0],self.arm_tcp_list[1],self.arm_tcp_list[2],
                self.arm_tcp_list[3],self.arm_tcp_list[4],self.arm_tcp_list[5]  
            ],
            "joint": [
                self.arm_joint_list[0],self.arm_joint_list[1],self.arm_joint_list[2],
                self.arm_joint_list[3],self.arm_joint_list[4],self.arm_joint_list[5]  
            ]
        }
        
        #将单次的json字典添加进buffer列表
        self.json_data_buffer.append(json_dic)
        
        start_time = time.time()

        if len(self.json_data_buffer) > 20:
            #将json buffer中的内容写进json文件
            with open(self.json_file_path, 'a') as json_file:
                json.dump(self.json_data_buffer, json_file, indent=4)
                end_time = time.time()
                elapsed_time = end_time - start_time
                rospy.loginfo(f"成功保存json文件,耗时: {elapsed_time} 秒")
                #清空json buffer
                self.json_data_buffer.clear()
            

    def arm_state_callback(self,data):
        # 标志位
        if self.get_arm_state_flag == False:
            self.get_arm_state_flag = True
        # 清空列表
        self.arm_tcp_list.clear()
        self.arm_joint_list.clear()
        # 填充新状态
        for tcp in data.cur_tcp_pose:
            self.arm_tcp_list.append(tcp)
        for joint in data.cur_joints_pose:
            self.arm_joint_list.append(joint)
    
def main():
    
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description="")
    # 添加参数
    parser.add_argument('--data_name', type=str, help='数据存储位置', required=True)

    # 解析命令行参数
    args = parser.parse_args()
    
    """
    订阅自定义消息并发布 JointState 消息的函数
    """
    rospy.init_node('normal_data_save_node', anonymous=True)
    rospy.loginfo("init normal data save node")
    
    data_saver_node = data_saver(args)
    
    rospy.spin()

if __name__ == '__main__':
    main()