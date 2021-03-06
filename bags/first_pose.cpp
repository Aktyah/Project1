#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
//#include "pub_sub/Num.h"
//#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

//include libraries for tf
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

//include the service header file
#include "Prj1/pose.h"

class FirstPose
{
public:
  FirstPose(){
    first_pose_sub = n.subscribe <geometry_msgs::PoseStamped>("/robot/pose", 1000, &FirstPose::readerCallback, this);
    //first_pose_pub = n.advertise <geometry_msgs::PoseStamped>("first_pose", 1000);

    service = n.advertiseService("reset", &FirstPose::resetCallback, this);

  }

private:
  ros::NodeHandle n;
  ros::Subscriber first_pose_sub;
  //ros::Publisher first_pose_pub;

  ros::ServiceServer service; 

  bool flag = 1;

  //geometry_msgs::Pose pose;
  //std_msgs::Header header;

  geometry_msgs::PoseStamped stamped;

  void readerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    stamped.header = msg->header;
    stamped.pose = msg->pose;

  if(flag==1){

    std::string odom = "odom";
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.frame_id = "world";
    static_transformStamped.header.stamp = stamped.header.stamp;
    static_transformStamped.header.seq = stamped.header.seq;
    static_transformStamped.child_frame_id = odom;
    static_transformStamped.transform.translation.x = stamped.pose.position.x;
    static_transformStamped.transform.translation.y = stamped.pose.position.y;
    static_transformStamped.transform.translation.z = stamped.pose.position.z;
    static_transformStamped.transform.rotation.x = stamped.pose.orientation.x;
    static_transformStamped.transform.rotation.y = stamped.pose.orientation.y;
    static_transformStamped.transform.rotation.z = stamped.pose.orientation.z;
    static_transformStamped.transform.rotation.w = stamped.pose.orientation.w;
    static_broadcaster.sendTransform(static_transformStamped);

    //first_pose_pub.publish(stamped);

    flag = 0;
    }

  }

  bool resetCallback(Prj1::pose::Request  &req, Prj1::pose::Response &res) {
    //req.x;
    //req.y;
    //req.theta;

    std::string odom = "odom";
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.frame_id = "world";
    static_transformStamped.header.stamp = stamped.header.stamp;
    static_transformStamped.header.seq = stamped.header.seq;
    static_transformStamped.child_frame_id = odom;
    static_transformStamped.transform.translation.x = req.x;;
    static_transformStamped.transform.translation.y = req.y;
    static_transformStamped.transform.translation.z = 0;
    //from angle to quaternion
    tf2::Quaternion q;
    double yaw = req.theta;
    q.setRPY(0, 0, yaw);
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(static_transformStamped);

    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "first_pose");
  FirstPose FirstPose;
  ros::spin();
  return 0;
}
