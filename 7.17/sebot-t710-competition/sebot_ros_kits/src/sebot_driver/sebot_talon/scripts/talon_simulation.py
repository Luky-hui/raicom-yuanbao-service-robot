#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@file 		   :talon_simulation.py
@Description   :通过坐标位置来机械臂
@Date 		   :2024/03/28 19:29:59
@Autor         :HC
@Version       :v1.0
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped

class MoveItArm:
    """
    moveit机械臂控制
    """
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)	 # 初始化moveit_commander，并启动moveit节点
        rospy.init_node('sebot_talon')                   # 节点名称            

        self.arm = moveit_commander.MoveGroupCommander('arm')           # 机械臂主体部分 
        # self.gripper = moveit_commander.MoveGroupCommander('gripper') # 机械爪部分
        
        self.reference_frame = 'base_link'          # 设置目标位置所使用的参考坐标系        
        
        self.arm.set_pose_reference_frame(self.reference_frame)  # 定位坐标系
        
        self.arm.allow_replanning(True)                    # 是否允许重复规划路径(最多五次)             

        self.arm.set_goal_joint_tolerance(0.3)             # 设置位置(单位：米)允许误差

        self.arm.set_start_state_to_current_state()        # 设置机器臂当前的状态作为运动初始状态

        self.arm.set_named_target('home')  # 控制机械臂先回到初始化位置
        self.arm.go()
        rospy.sleep(1)

    def coordCtrl(self,target_pose):
        """
        通过坐标控制机械臂
        param: target_pose 目标位姿
        """        
        self.arm.set_pose_target(target_pose, self.arm.get_end_effector_link())  # 获取机械臂末端运动目标位姿
       
        traj = self.arm.plan()    # 规划运动路径
        self.arm.execute(traj)    # 执行运动路径         
        rospy.sleep(2)            # 等待运动完成
        


if __name__ == "__main__":
    moveItArm = MoveItArm()
    # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
    # 设置机械臂工作空间中的目标位姿
    target_pose = PoseStamped()
    target_pose.header.frame_id = moveItArm.reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = -0.0996125
    target_pose.pose.position.y = -0.0204184
    target_pose.pose.position.z = 0.109637
    target_pose.pose.orientation.x = -0.297784
    target_pose.pose.orientation.y = -0.642886
    target_pose.pose.orientation.z = 0.643069
    target_pose.pose.orientation.w = -0.290663

    moveItArm.coordCtrl(target_pose)    # 通过坐标控制机械臂

    # 关闭并退出moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


