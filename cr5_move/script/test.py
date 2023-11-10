#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import random 
import actionlib
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
 
 
 
class MoveItIkDemo:
    def __init__(self):
 
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

    def random_move(self):
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('cr5_arm')
                
        # 获取终端link的名称，这个在setup assistant中设置过了
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
 
        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(1)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        #参考坐标系，前面设置了
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now() #时间戳？
    
        pose_cur = arm.get_current_pose()
        rpy_cur = arm.get_current_rpy()

        for i in range(10):


            # print(pose_cur)
            # print(rpy_cur)

            #末端位置   
            target_pose.pose.position.x = pose_cur.pose.position.x-random.random()/50
            target_pose.pose.position.y = pose_cur.pose.position.y-random.random()/50
            target_pose.pose.position.z = pose_cur.pose.position.z-random.random()/50
            #末端姿态，四元数
            target_pose.pose.orientation.x = pose_cur.pose.orientation.x
            target_pose.pose.orientation.y = pose_cur.pose.orientation.y
            target_pose.pose.orientation.z = pose_cur.pose.orientation.z
            target_pose.pose.orientation.w = pose_cur.pose.orientation.w
            
            # 设置机器臂当前的状态作为运动初始状态
            arm.set_start_state_to_current_state()

            # 设置机械臂终端运动的目标位姿
            arm.set_pose_target(target_pose, end_effector_link)
            
            # 规划运动路径，返回虚影的效果
            traj = arm.plan()
            
            # 按照规划的运动路径控制机械臂运动
            arm.execute(traj)
            # rospy.sleep(1)  #执行完成后休息1s
    

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
 


    def move(self):
        # 机械臂中joint的命名
        arm_joints = ['joint1',
                      'joint2',
                      'joint3', 
                      'joint4',
                      'joint5',
                      'joint6']
        
        arm_goal  = [0.0, -0.2, math.pi/2, 0.0, 0.0, 0.0]
    
        # 连接机械臂轨迹规划的trajectory action server
        rospy.loginfo('Waiting for arm trajectory controller...')       
        arm_client = actionlib.SimpleActionClient('cr5_robot/joint_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        arm_client.wait_for_server()        
        rospy.loginfo('...connected.')  
    
        # 使用设置的目标位置创建一条轨迹数据
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)

        print(arm_trajectory.points)
    
        rospy.loginfo('Moving the arm to goal position...')
        
        # 创建一个轨迹目标的空对象
        arm_goal = FollowJointTrajectoryGoal()
        
        # 将之前创建好的轨迹数据加入轨迹目标对象中
        arm_goal.trajectory = arm_trajectory
        
        # 设置执行时间的允许误差值
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # 将轨迹目标发送到action server进行处理，实现机械臂的运动控制
        arm_client.send_goal(arm_goal)

        # 等待机械臂运动结束
        arm_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
 
if __name__ == "__main__":
    it = MoveItIkDemo()
    it.move()
    it.random_move()
    # 关闭并退出moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
