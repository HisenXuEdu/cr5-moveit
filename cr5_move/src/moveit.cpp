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
#include "cartesian_state_msgs/PoseTwist.h"



void pub_status(moveit::planning_interface::MoveGroupInterface& move_group){
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    // 获取当前欧拉角信息
    const std::vector<double> current_rpy = move_group.getCurrentRPY();
    const geometry_msgs::TwistStamped current_end_effector_velocity = move_group.getEndEffectorVelocity();

    cartesian_state_msgs::PoseTwist> > msg;


    msg->pose.position.x = current_pose.pose.position.x;
    msg->pose.position.y = current_pose.pose.position.y; 
    msg->pose.position.z = current_pose.pose.position.z;

    msg->pose.orientation.x = current_pose.pose.orientation.x;
    msg->pose.orientation.y = current_pose.pose.orientation.y;
    msg->pose.orientation.z = current_pose.pose.orientation.z;
    msg->pose.orientation.w = current_pose.pose.orientation.w;

    msg->twist.linear.x = current_end_effector_velocity.twist.linear.x;
    msg->twist.linear.y = current_end_effector_velocity.twist.linear.y;
    msg->twist.linear.z = current_end_effector_velocity.twist.linear.z;
    msg->twist.angular.x = current_end_effector_velocity.twist.angular.x;
    msg->twist.angular.y = current_end_effector_velocity.twist.angular.y;
    msg->twist.angular.z = current_end_effector_velocity.twist.angular.z;

    pub_status.publish(msg);
}

void moveit_control(
  const geometry_msgs::Twist arm_twist_cmd) {
    trajectory_msgs::JointTrajectory arm_trajectory;
    arm_trajectory.joint_names = arm_joints;
    arm_trajectory.points.resize(1);
    arm_trajectory.points[0].positions.resize(arm_joints.size(), 0.0);

    arm_trajectory.points[0].velocities[0] = arm_twist_cmd.linear.x
    arm_trajectory.points[0].velocities[1] = arm_twist_cmd.linear.y
    arm_trajectory.points[0].velocities[2] = arm_twist_cmd.linear.z
    arm_trajectory.points[0].velocities[3] = arm_twist_cmd.angular.x
    arm_trajectory.points[0].velocities[4] = arm_twist_cmd.angular.y
    arm_trajectory.points[0].velocities[5] = arm_twist_cmd.angular.z 

    arm_trajectory.points[0].accelerations.resize(arm_joints.size(), 0.0);
    arm_trajectory.points[0].time_from_start = ros::Duration(3.0);
    geometry_msgs::Twist arm_twist_cmd;
    arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0);
    arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1);
    arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2);
    arm_twist_cmd.angular.x = arm_desired_twist_adm_(3);
    arm_twist_cmd.angular.y = arm_desired_twist_adm_(4);
    arm_twist_cmd.angular.z = arm_desired_twist_adm_(5);

}

void pub(ros::NodeHandle& nh, ros::Publisher& pub, moveit::planning_interface::MoveGroupInterface& move_group){
    ROS_INFO("pub status .................");
    ros::Rate loop_rate_;
    while (nh.ok()) {
        pub_status(move_group)
        ros::spinOnce();
        loop_rate_.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_control");
    ros::NodeHandle nh;
    pub_status = nh.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

    moveit::planning_interface::MoveGroupInterface move_group("cr5_arm");
    
    pub(nh, pub_status, move_group);


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