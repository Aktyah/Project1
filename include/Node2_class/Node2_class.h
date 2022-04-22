#ifndef NODE2_CLASS
#define NODE2_CLASS

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// This class is supposed to be simply run (with the public method run_node)
// It'll then perform odometry automatically from teh data that
// comes from the first node. Use dynamic reconfigure and services to change the settings
class Node2
{

    // Time
    double T1 = 0;
    // Pose of the robot
    nav_msgs::Odometry current_pose;
    // Velocity read from /cmd_vel
    geometry_msgs::TwistStamped velocity;
    // Broadcaster data
    geometry_msgs::TransformStamped transformStamped; //
    // Integration method
    int mode;
    // Publications
    ros::Publisher Odom_publisher;
    // Subscriber
    ros::Subscriber reader;
    // TF2 broadcaster
    tf2_ros::TransformBroadcaster br; //
    // Service
    ros::ServiceServer service;
    // Node handle
    ros::NodeHandle n;
    // Dynamic reconfigure declarations
    dynamic_reconfigure::Server<parNode2::parametersConfig> dynServer;
    dynamic_reconfigure::Server<parNode2::parametersConfig>::CallbackType call;

    // Dynamic configure function (Odometry integration method choice)
    void callbackConf(parNode2::parametersConfig &con);

    // Static configure of the first pose
    void setFirstPose(ros::NodeHandle n);

    // Service Callback
    bool pose_callback(Prj1::pose::Request &req, Prj1::pose::Response &res);

    // sub's callback, reads the velocities coming from /cmd_vel
    void readVel(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Integration (Euler and RK)
    void Integration(int &mode);

    // Broadcast the transformation between odom and base_link
    void callback_tf2(const nav_msgs::Odometry &msg);

    //********************************************************************************************************************
public:
    Node2(); // we must define the class constructor

    // Main function to run
    void run_node();
};

#endif