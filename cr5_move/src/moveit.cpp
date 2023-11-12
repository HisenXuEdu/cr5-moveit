#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
 
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
 
#include<iostream>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_control");
    ros::NodeHandle nh;

    // 打印日志消息
    ROS_INFO("Waiting for arm trajectory controller...");


    // 创建动作客户端
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client(
        "cr5_robot/joint_controller/follow_joint_trajectory", true);

    // 等待动作服务器连接
    arm_client.waitForServer();

    // 使用设置的目标位置创建一条轨迹数据
    trajectory_msgs::JointTrajectory arm_trajectory;
    arm_trajectory.joint_names = arm_joints;
    arm_trajectory.points.resize(1);
    arm_trajectory.points[0].positions = arm_goal;
    arm_trajectory.points[0].velocities.resize(arm_joints.size(), 0.0);
    arm_trajectory.points[0].accelerations.resize(arm_joints.size(), 0.0);
    arm_trajectory.points[0].time_from_start = ros::Duration(3.0);

    // 打印轨迹点信息（可选）
    for (int i=0; i < arm_trajectory.points[0].positions.size(); i++)
    {
        ROS_INFO_STREAM("Joint " << arm_trajectory.joint_names[i] << ": " << arm_trajectory.points[0].positions[i]);
    }

    // 打印日志消息
    ROS_INFO("Moving the arm to goal position...");

    // 创建一个轨迹目标的空对象
    control_msgs::FollowJointTrajectoryGoal arm_goal;

    // 将之前创建好的轨迹数据加入轨迹目标对象中
    arm_goal.trajectory = arm_trajectory;

    // 设置执行时间的允许误差值
    arm_goal.goal_time_tolerance = ros::Duration(0.0);

    // 将轨迹目标发送到action server进行处理，实现机械臂的运动控制
    arm_client.sendGoal(arm_goal);

    // 等待机械臂运动结束
    arm_client.waitForResult(ros::Duration(5.0));

    return 0;
}