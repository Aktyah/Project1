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
#include "std_msgs/Float64.h"
#include <dynamic_reconfigure/server.h>
#include "Prj1/parametersConfig.h"
#include "Prj1/pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>
#include <iostream>
#include <cstdio>

#include "Node2_class/Node2_class.h"

Node2::Node2()
{
    this->reader = this->n.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1000, &Node2::readVel, this);
    this->Odom_publisher = this->n.advertise<nav_msgs::Odometry>("odom", 1000);
    this->service = this->n.advertiseService("new_pose", &Node2::pose_callback, this);
    this->call = boost::bind(&Node2::callbackConf, this, _1);
    this->first_pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/robot/pose", 1000, &Node2::reset_bag, this);
}

void Node2::callbackConf(parNode2::parametersConfig &con)
{
    mode = con.method;
    ROS_INFO("**** INTEGRATING WITH %s ****", mode ? "RK" : "Euler");
}

void Node2::setFirstPose(ros::NodeHandle n)
{
    current_pose.header.frame_id = "odom"; // frame_id in wich odometry is done
    this->n.getParam("FirstPoseX", current_pose.pose.pose.position.x);
    this->n.getParam("FirstPoseY", current_pose.pose.pose.position.y);
    this->n.getParam("FirstPoseZ", current_pose.pose.pose.position.z);
    this->n.getParam("FirstPoserw", current_pose.pose.pose.orientation.w);
    this->n.getParam("FirstPoserx", current_pose.pose.pose.orientation.x);
    this->n.getParam("FirstPosery", current_pose.pose.pose.orientation.y);
    this->n.getParam("FirstPoserz", current_pose.pose.pose.orientation.z);
}

bool Node2::pose_callback(Prj1::pose::Request &req, Prj1::pose::Response &res)
{
    nav_msgs::Odometry current_pose_world;
    // set header
    current_pose_world.header.frame_id = current_pose.header.frame_id;

    // Set the new X Y coord's
    current_pose_world.pose.pose.position.x = req.x;
    current_pose_world.pose.pose.position.y = req.y;

    // Set the new theta with quaternions
    tf2::Quaternion quat;
    double roll, pitch, yaw;
    // old pose
    quat.setX(current_pose_world.pose.pose.orientation.x);
    quat.setY(current_pose_world.pose.pose.orientation.y);
    quat.setZ(current_pose_world.pose.pose.orientation.z);
    quat.setW(current_pose_world.pose.pose.orientation.w);
    quat.setRPY(roll, pitch, yaw);

    // set new pose (with the new yaw angle)
    yaw = req.theta;
    quat.setRPY(roll, pitch, yaw);
    current_pose_world.pose.pose.orientation.x = quat.getX();
    current_pose_world.pose.pose.orientation.y = quat.getY();
    current_pose_world.pose.pose.orientation.z = quat.getZ();
    current_pose_world.pose.pose.orientation.w = quat.getW();

    Node2::callback_tf2(current_pose_world);

    current_pose.pose.pose.position.x = 0;
    current_pose.pose.pose.position.y = 0;

    tf2::Quaternion q;
    q.setEuler(0, 0, 0);
    current_pose.pose.pose.orientation.x = q.getX();
    current_pose.pose.pose.orientation.y = q.getY();
    current_pose.pose.pose.orientation.z = q.getZ();
    current_pose.pose.pose.orientation.w = q.getW();

    ROS_INFO("NEW POSE:  %f  %f  %f", req.x, req.y, req.theta);
    return true;
}

void Node2::readVel(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Header
    velocity.header.seq = msg->header.seq;
    velocity.header.stamp.sec = msg->header.stamp.sec;
    velocity.header.stamp.nsec = msg->header.stamp.nsec;
    velocity.header.frame_id = msg->header.frame_id;
    // Twist
    velocity.twist.linear.x = msg->twist.linear.x;
    velocity.twist.linear.y = msg->twist.linear.y;
    velocity.twist.linear.z = msg->twist.linear.z;
    velocity.twist.angular.x = msg->twist.angular.x;
    velocity.twist.angular.y = msg->twist.angular.y;
    velocity.twist.angular.z = msg->twist.angular.z;
}

void Node2::Integration(int &mode)
{
    // Header
    current_pose.header.seq = velocity.header.seq;
    //  current_pose.header.frame_id = "odom"; // frame in wich odometry is done
    // dT definition and definition of T1 for the next iteration
    double t1 = velocity.header.stamp.sec;
    double t2 = velocity.header.stamp.nsec;
    double T2 = t1 + t2 * 1e-9;
    double dT = T2 - T1;

    T1 = T2;
    current_pose.header.stamp.sec = T1 - t2 * 1e-9;
    current_pose.header.stamp.nsec = (T1 - t1) * 1e9;
    // Child frame Id
    current_pose.child_frame_id = velocity.header.frame_id; // robot's local rs
    // Transformation from quaternions to euler angles
    tf2::Quaternion quat_current;
    quat_current.setX(current_pose.pose.pose.orientation.x);
    quat_current.setY(current_pose.pose.pose.orientation.y);
    quat_current.setZ(current_pose.pose.pose.orientation.z);
    quat_current.setW(current_pose.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat_current).getRPY(roll, pitch, yaw);
    // Angle related computations
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    yaw = yaw + velocity.twist.angular.z * dT;

    // Twist
    current_pose.twist.twist.linear.x = velocity.twist.linear.x;
    current_pose.twist.twist.linear.y = velocity.twist.linear.y;
    current_pose.twist.twist.linear.z = velocity.twist.linear.z;
    current_pose.twist.twist.angular.x = velocity.twist.angular.x;
    current_pose.twist.twist.angular.y = velocity.twist.angular.y;
    current_pose.twist.twist.angular.z = velocity.twist.angular.z;

    // New quaternions due to the new yaw angle
    quat_current.setRPY(roll, pitch, yaw);
    if (integrate == 2){
    
        // Pose
        switch (mode)
        {
        case 0:
            current_pose.pose.pose.position.x = current_pose.pose.pose.position.x + (velocity.twist.linear.x * cos_yaw - velocity.twist.linear.y * sin_yaw) * dT;
            current_pose.pose.pose.position.y = current_pose.pose.pose.position.y + (velocity.twist.linear.x * sin_yaw + velocity.twist.linear.y * cos_yaw) * dT;
            current_pose.pose.pose.position.z = current_pose.pose.pose.position.z + velocity.twist.linear.z * dT;
            break;
        case 1:
        {
            double theta = yaw + (velocity.twist.angular.z * dT) / 2;
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);
            current_pose.pose.pose.position.x = current_pose.pose.pose.position.x + (velocity.twist.linear.x * cos_theta - velocity.twist.linear.y * sin_theta) * dT;
            current_pose.pose.pose.position.y = current_pose.pose.pose.position.y + (velocity.twist.linear.x * sin_theta + velocity.twist.linear.y * cos_theta) * dT;
            current_pose.pose.pose.position.z = current_pose.pose.pose.position.z + velocity.twist.linear.z * dT;
            break;
        }
        default:
            ROS_INFO("!! Something went wrong, no integration method is selected !!");
            break;
        }
        current_pose.pose.pose.orientation.x = quat_current.getX();
        current_pose.pose.pose.orientation.y = quat_current.getY();
        current_pose.pose.pose.orientation.z = quat_current.getZ();
        current_pose.pose.pose.orientation.w = quat_current.getW();
    }
    else
    {
        integrate = integrate +1;
    }
    // Covariance is left to 0

    // set header
    transformStamped_2.header.stamp = current_pose.header.stamp;
    transformStamped_2.header.frame_id = "odom";
    transformStamped_2.child_frame_id = "base_link";
    // set x,y
    transformStamped_2.transform.translation.x = current_pose.pose.pose.position.x;
    transformStamped_2.transform.translation.y = current_pose.pose.pose.position.y;
    transformStamped_2.transform.translation.z = current_pose.pose.pose.position.z;
    // set theta
    transformStamped_2.transform.rotation.x = current_pose.pose.pose.orientation.x;
    transformStamped_2.transform.rotation.y = current_pose.pose.pose.orientation.y;
    transformStamped_2.transform.rotation.z = current_pose.pose.pose.orientation.z;
    transformStamped_2.transform.rotation.w = current_pose.pose.pose.orientation.w;
    // send transform
    br2.sendTransform(transformStamped_2);
}

void Node2::callback_tf2(const nav_msgs::Odometry &msg)
{
    // set header
    transformStamped.header.stamp = msg.header.stamp;
    transformStamped.header.frame_id = main_frame_id;
    transformStamped.child_frame_id = msg.header.frame_id;
    // set x,y
    transformStamped.transform.translation.x = msg.pose.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.pose.position.z;
    // set theta
    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
    // send transform
    br.sendTransform(transformStamped);
}

void Node2::reset_bag(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (msg->header.seq != old_seq + 1)
    {
        stamped.header = msg->header;
        stamped.pose = msg->pose;

        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped static_transformStamped;
        tf2::Quaternion q;

        static_transformStamped.header.frame_id = main_frame_id;
        static_transformStamped.child_frame_id = current_pose.header.frame_id;
        static_transformStamped.transform.translation.x = stamped.pose.position.x;
        static_transformStamped.transform.translation.y = stamped.pose.position.y;
        static_transformStamped.transform.translation.z = stamped.pose.position.z;

        q.setX(stamped.pose.orientation.x);
        q.setY(stamped.pose.orientation.y);
        q.setZ(stamped.pose.orientation.z);
        q.setW(stamped.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        q.setEuler(0, 0, yaw);

        static_transformStamped.transform.rotation.x = q.getX();
        static_transformStamped.transform.rotation.y = q.getY();
        static_transformStamped.transform.rotation.z = q.getZ();
        static_transformStamped.transform.rotation.w = q.getW();

        static_broadcaster.sendTransform(static_transformStamped);

        q.setEuler(0, 0, 0);
        current_pose.pose.pose.orientation.x = q.getX();
        current_pose.pose.pose.orientation.y = q.getY();
        current_pose.pose.pose.orientation.z = q.getZ();
        current_pose.pose.pose.orientation.w = q.getW();
        current_pose.pose.pose.position.x = 0;
        current_pose.pose.pose.position.y = 0;
        current_pose.pose.pose.position.z = 0;
        integrate = 0;
    }
    old_seq = msg->header.seq;
}

void Node2::run_node()
{
    setFirstPose(n);
    dynServer.setCallback(call);
    callback_tf2(current_pose);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        Node2::Integration(mode);
        Node2::Odom_publisher.publish(current_pose);
        ros::spinOnce();
    }
}