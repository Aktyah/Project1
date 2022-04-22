#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "pub_sub/Num.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"


class FirstPose
{
public:
  FirstPose(){
    first_pose_sub = n.subscribe <geometry_msgs::PoseStamped>("/robot/pose", 1000, &FirstPose::readerCallback, this);
    first_pose_pub = n.advertise <geometry_msgs::PoseStamped>("first_pose", 1000);

  }

  void readerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  if(flag==1){
    stamped.header = msg->header;
    stamped.pose = msg->pose;

    first_pose_pub.publish(stamped);

    flag = 0;
  }

}

private:
  ros::NodeHandle n;
  ros::Subscriber first_pose_sub;
  ros::Publisher first_pose_pub;

  bool flag = 1;

  geometry_msgs::Pose pose;
  std_msgs::Header header;

  geometry_msgs::PoseStamped stamped;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "first_pose");
  FirstPose FirstPose;
  ros::spin();
  return 0;
}
