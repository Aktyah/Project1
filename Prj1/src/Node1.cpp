#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pub_sub/Num.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

#include <sstream>



double r;
double l;
double w;

bool flag=1;
int past_sec;
int past_nsec;
double past_position_1;
double past_position_2;
double past_position_3;
double past_position_4;

void readerCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  ROS_INFO("wheel state received");

  float position_1 = msg->position[0];
  float position_2 = msg->position[1];
  float position_3 = msg->position[2];
  float position_4 = msg->position[3];

  int sec = msg->header.stamp.sec;
  int nsec = msg->header.stamp.nsec;

  //ROS_INFO("r: [%f]", r);
  //ROS_INFO("l: [%f]", l);
  //ROS_INFO("w: [%f]", w);
  
  int delta_sec;
  float delta_nsec;
  float delta_t;

  float delta_pos_1;
  float delta_pos_2;
  float delta_pos_3;
  float delta_pos_4;

  //ROS_INFO("flag: [%d]", flag);

  //ROS_INFO("Position_1: [%f]", position_1);
  //ROS_INFO("Position_2: [%f]", position_2);
  //ROS_INFO("Position_3: [%f]", position_3);
  //ROS_INFO("Position_4: [%f]", position_4);

  if(flag==1){
    past_sec = sec;
    past_nsec = nsec;
    past_position_1 = position_1;
    past_position_2 = position_2;
    past_position_3 = position_3;
    past_position_4 = position_4;

    flag = 0;
  }
  else{
    delta_sec = sec-past_sec;
    delta_nsec = (nsec-past_nsec)*0.000000001;

    delta_t = delta_nsec+ (float) delta_sec;

    //ROS_INFO("delta_t: [%f]", delta_t);

    delta_pos_1 = position_1-past_position_1;
    delta_pos_2 = position_2-past_position_2;
    delta_pos_3 = position_3-past_position_3;
    delta_pos_4 = position_4-past_position_4;

    //ROS_INFO("delta_pos_1: [%f]", delta_pos_1);

    float u1 = delta_pos_1/delta_t;
    ROS_INFO("u1: [%f]", u1);
    float u2 = delta_pos_2/delta_t;
    ROS_INFO("u2: [%f]", u2);
    float u3 = delta_pos_3/delta_t;
    ROS_INFO("u3: [%f]", u3);
    float u4 = delta_pos_4/delta_t;
    ROS_INFO("u4: [%f]", u4);

    float w_bz = r/4/(l+w)*(-u1+u2+u3-u4);
    float v_bx = r/4*(u1+u2+u3+u4);
    float v_by = r/4*(-u1+u2-u3+u4);

    ROS_INFO("w_bz: [%f]", w_bz);
    ROS_INFO("v_bx: [%f]", v_bx);
    ROS_INFO("v_by: [%f]", v_by);

    past_position_1 = position_1;
    past_position_2 = position_2;
    past_position_3 = position_3;
    past_position_4 = position_4;
    past_sec = sec;
    past_nsec = nsec;
  }

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  //past_sec = 1648570202; //first value on the bag
  //past_nsec = 312687158;

  n.getParam("/radius", r);
  n.getParam("/lenght", l);
  n.getParam("/width", w);
 

  ros::Subscriber sub_reader = n.subscribe <sensor_msgs::JointState>("wheel_states", 1000, readerCallback);
  
  ros::spin();

  return 0;
}
