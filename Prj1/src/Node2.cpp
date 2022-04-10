#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose2D.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include <dynamic_reconfigure/server.h>
#include <Prj1/parametersConfig.h>

#include <cmath>
#include <iostream>

// first state
nav_msgs::Odometry current_pose;
// Velocity of the robot
geometry_msgs::TwistStamped velocity;

// Reader's callback (it reads v and omega of the robot)
// namespace ConstPtr has the pointer version of the function
void readVel (const geometry_msgs::TwistStamped::ConstPtr& msg){
    //Header
    velocity.header.seq = msg->header.seq;
    velocity.header.stamp.sec = msg->header.stamp.sec;
    velocity.header.stamp.nsec = msg->header.stamp.nsec;
    velocity.header.frame_id = msg->header.frame_id;
    // Twist
    velocity.twist.linear.x  = msg->twist.linear.x;
    velocity.twist.linear.y  = msg->twist.linear.y;
    velocity.twist.linear.z  = msg->twist.linear.z;
    velocity.twist.angular.x = msg->twist.angular.x;
    velocity.twist.angular.y = msg->twist.angular.y;
    velocity.twist.angular.z = msg->twist.angular.z;
}

double T1 = 0;
void EulerOdom (tf::Quaternion& quat_input){
    // Header
    current_pose.header.seq = velocity.header.seq;
    current_pose.header.frame_id = "world";

    // dT definition and definition of T1 for the next iteration
    double t1 = velocity.header.stamp.sec;
    double t2 = velocity.header.stamp.nsec;
    double T2 = t1 + t2*1e-9; 
    double dT = T2 - T1;
    T1 = T2;
    current_pose.header.stamp.sec  = T1 - t2 * 1e-9;
    current_pose.header.stamp.nsec = (T1 - t1)*1e9;

    // Child frame Id
    current_pose.child_frame_id = "robot";

    // Transformation from quaternions to euler angles
    tfScalar roll, pitch, yaw;
    tf::Quaternion quat;
    tf::Matrix3x3(quat_input).getRPY(roll,pitch,yaw);
    
    // Angle related computations
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    yaw = yaw + velocity.twist.angular.z * dT;
    //ROS_INFO("%f", yaw);
    double vel = velocity.twist.linear.x*velocity.twist.linear.x + velocity.twist.linear.y*velocity.twist.linear.y;
    vel = sqrt(vel);

    // Twist
    current_pose.twist.twist.linear.x  = current_pose.twist.twist.linear.x  + velocity.twist.linear.x  * dT;
    current_pose.twist.twist.linear.y  = current_pose.twist.twist.linear.y  + velocity.twist.linear.y  * dT;
    current_pose.twist.twist.linear.z  = current_pose.twist.twist.linear.z  + velocity.twist.linear.z;
    current_pose.twist.twist.angular.x = current_pose.twist.twist.angular.x + velocity.twist.angular.x * dT; 
    current_pose.twist.twist.angular.y = current_pose.twist.twist.angular.y + velocity.twist.angular.y * dT;
    current_pose.twist.twist.angular.z = current_pose.twist.twist.angular.z + velocity.twist.angular.z * dT;

    // New quaternions due to the new yaw angle
    quat_input.setRPY(roll,pitch,yaw);
    // ROS_INFO("%f", current_pose.pose.pose.orientation.x);

    // Pose
    current_pose.pose.pose.position.x = current_pose.pose.pose.position.x + velocity.twist.linear.x * dT;
    current_pose.pose.pose.position.y = current_pose.pose.pose.position.y + velocity.twist.linear.y * dT;
    current_pose.pose.pose.position.z = current_pose.pose.pose.position.z + velocity.twist.linear.z;
    current_pose.pose.pose.orientation.x = quat_input.getX();
    current_pose.pose.pose.orientation.y = quat_input.getY();
    current_pose.pose.pose.orientation.z = quat_input.getZ();
    current_pose.pose.pose.orientation.w = quat_input.getW();
    // I'll leave Covariance blanc for now*******************
}

void callbackConf(nav_msgs::Odometry *curr_pose, parNode2::parametersConfig &con){
    ROS_INFO("THE POSE HAS BEEN CHANGED");
    curr_pose->pose.pose.position.x    = con.FirstPoseX;
    curr_pose->pose.pose.position.y    = con.FirstPoseY;
    curr_pose->pose.pose.orientation.x = con.FirstPosex;
    curr_pose->pose.pose.orientation.y = con.FirstPosey;
    curr_pose->pose.pose.orientation.z = con.FirstPosez;
    curr_pose->pose.pose.orientation.w = con.FirstPosew;
    ROS_INFO("%f", curr_pose->pose.pose.orientation.x);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Node2");
    ros::NodeHandle n;
    dynamic_reconfigure::Server<parNode2::parametersConfig> dynServer;
    dynamic_reconfigure::Server<parNode2::parametersConfig>::CallbackType call;
    call = boost::bind(&callbackConf, &current_pose, _1);
    dynServer.setCallback(call);
    tf::Quaternion quat_current;
    ros::Subscriber reader = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1000, readVel);
    ros::Publisher Odom_publisher = n.advertise<nav_msgs::Odometry>("odom",1000);
//    tf::Quaternion quat_current;

    ros::Rate loop_rate(10);
    while(ros::ok()){
        quat_current.setX(current_pose.pose.pose.orientation.x);
        quat_current.setY(current_pose.pose.pose.orientation.y);
        quat_current.setZ(current_pose.pose.pose.orientation.z);
        quat_current.setW(current_pose.pose.pose.orientation.w);
        EulerOdom(quat_current);
        Odom_publisher.publish(current_pose);

        ros::spinOnce();

    }
}