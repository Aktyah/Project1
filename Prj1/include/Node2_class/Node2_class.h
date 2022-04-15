#ifndef NODE2_CLASS
#define NODE2_CLASS

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

// This class is supposed to be simply run (with the public method run_node)
// It'll then perform odometry automatically from teh data that
// comes from the first node. Use dynamic reconfigure and services to change the settings
class Node2{

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


// Dynamic configure function (Odometry integration method choice)
void callbackConf (parNode2::parametersConfig &con);

// Static configure of the first pose
void setFirstPose (ros::NodeHandle n);

// Service Callback
bool pose_callback (Prj1::pose::Request &req , Prj1::pose::Response &res);

// sub's callback, reads the velocities coming from /cmd_vel
void readVel (const geometry_msgs::TwistStamped::ConstPtr& msg);

// Integration (Euler and RK)
void Integration (int &mode);

//********************************************************************************************************************
public:
Node2(); // we must define the class constructor
// Main function to run
void run_node();
};


#endif