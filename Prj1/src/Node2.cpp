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
#include "Prj1/parametersConfig.h"
#include "Prj1/pose.h"

#include <cmath>
#include <iostream>

class Node2{

// *********
// Variables
// *********

// Time 
double T1 = 0;
// Pose of the robot
nav_msgs::Odometry current_pose;
// Velocity read from /cmd_vel
geometry_msgs::TwistStamped velocity;
// Integration method
int mode;
// Publications
ros::Publisher Odom_publisher;
// Subscriber
ros::Subscriber reader;
// Service
ros::ServiceServer service;
// Node handle
ros::NodeHandle n;
// Dynamic reconfigure declarations
dynamic_reconfigure::Server<parNode2::parametersConfig> dynServer;
dynamic_reconfigure::Server<parNode2::parametersConfig>::CallbackType call; 

// *********
// Functions
// *********

// Dynamic configure function (Odometry integration method choice)
void callbackConf (parNode2::parametersConfig &con){
    mode = con.selection;
    ROS_INFO("**** INTEGRATING WITH %s ****", mode?"RK":"Euler");
}

// Static configure of the first pose
void setFirstPose (ros::NodeHandle n){
    this->n.getParam("FirstPoseX" , current_pose.pose.pose.position.x);
    this->n.getParam("FirstPoseY" , current_pose.pose.pose.position.y);
    this->n.getParam("FirstPoseZ" , current_pose.pose.pose.position.z);
    this->n.getParam("FirstPoserw", current_pose.pose.pose.orientation.w); 
    this->n.getParam("FirstPoserx", current_pose.pose.pose.orientation.x);
    this->n.getParam("FirstPosery", current_pose.pose.pose.orientation.y);
    this->n.getParam("FirstPoserz", current_pose.pose.pose.orientation.z);
}

// Service Callback
void pose_callback (Prj1::pose::Request &req){
    current_pose.pose.pose.position.x = req.x; 
    current_pose.pose.pose.position.y = req.y;
    current_pose.pose.pose.position.z = req.theta; 
}

// sub's callback, reads the velocities coming from /cmd_vel
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

// Integration (Euler and RK)
void Integration (int &mode){
    tf::Quaternion quat_current;
    quat_current.setX(current_pose.pose.pose.orientation.x);
    quat_current.setY(current_pose.pose.pose.orientation.y);
    quat_current.setZ(current_pose.pose.pose.orientation.z);
    quat_current.setW(current_pose.pose.pose.orientation.w);
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
    current_pose.child_frame_id = "base";
    // Transformation from quaternions to euler angles
    tfScalar roll, pitch, yaw;
    tf::Matrix3x3(quat_current).getRPY(roll,pitch,yaw);
    // Angle related computations
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    yaw = yaw + velocity.twist.angular.z * dT;

    // Twist
    current_pose.twist.twist.linear.x  = velocity.twist.linear.x;
    current_pose.twist.twist.linear.y  = velocity.twist.linear.x; 
    current_pose.twist.twist.linear.z  = velocity.twist.linear.z;
    current_pose.twist.twist.angular.x = velocity.twist.angular.x; 
    current_pose.twist.twist.angular.y = velocity.twist.angular.y;
    current_pose.twist.twist.angular.z = velocity.twist.angular.z;
    
    // New quaternions due to the new yaw angle
    quat_current.setRPY(roll,pitch,yaw);
    
    // Pose
    switch (mode){
        case 0:
            current_pose.pose.pose.position.x = current_pose.pose.pose.position.x + (velocity.twist.linear.x * cos_yaw + velocity.twist.linear.y * sin_yaw) * dT;
            current_pose.pose.pose.position.y = current_pose.pose.pose.position.y + (velocity.twist.linear.x * sin_yaw + velocity.twist.linear.y * cos_yaw) * dT;
            current_pose.pose.pose.position.z = current_pose.pose.pose.position.z + velocity.twist.linear.z;
            break;
        case 1:
        {
            double theta = yaw + (velocity.twist.angular.z*dT)/2;
            double cos_theta = std::abs(cos(theta));
            double sin_theta = std::abs(sin(theta));
            current_pose.pose.pose.position.x = current_pose.pose.pose.position.x + (velocity.twist.linear.x * cos_yaw * cos_theta + velocity.twist.linear.y * sin_yaw * sin_theta) * dT;
            current_pose.pose.pose.position.y = current_pose.pose.pose.position.y + (velocity.twist.linear.x * sin_yaw * cos_theta + velocity.twist.linear.y * cos_yaw * sin_theta) * dT;
            current_pose.pose.pose.position.z = current_pose.pose.pose.position.z + velocity.twist.linear.z;
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
    
    // Covariance is left to 0
}   

//********************************************************************************************************************

public:
Node2(){
    this->reader = this->n.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1000, &Node2::readVel, this);
    this->Odom_publisher = this->n.advertise<nav_msgs::Odometry>("odom",1000);
    this->service = this->n.advertiseService<Prj1::pose::Request>("new_pose", &Node2::pose_callback, this);
    this->call = boost::bind(&Node2::callbackConf, this, _1);
}

// Main function to run
void run_node(){
    dynServer.setCallback(call);
    setFirstPose(n);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        Integration(mode);
        Odom_publisher.publish(current_pose);
        ros::spinOnce();
    }
}

};

int main(int argc, char **argv){
    ros::init(argc, argv, "Node2");
    Node2 node;
    node.run_node();
}