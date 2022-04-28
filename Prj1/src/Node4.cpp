
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "geometry_msgs/TwistStamped.h"
#include "Prj1/wheels.h"
#include "math.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <iostream>
#include <fstream>

#define N 100000

class Subscriber
{
public:
  Subscriber()
  {
    velocity_bag = n.subscribe<sensor_msgs::JointState>("wheel_states", 1000, &Subscriber::Callback,this);
    velocity_sub = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1000, &Subscriber::chatterCallback,this);

    n.getParam("/radius", r);
    n.getParam("/lenght", l);
    n.getParam("/width",  w);
    fout1.open("v_bag.txt");
    fout2.open("v_computed.txt");
    i=0;
    flag1 = 1;
    flag2 = 1;
  }

  void Callback(const sensor_msgs::JointState::ConstPtr& msg_bag)
  {
    if(flag1 == 1 && flag2 == 1) {
      flag1 = 0;
       } 
    else if(flag1 == 0 && flag2 == 1){
      flag2 = 0;
       } 
     else {
      u1_bag=msg_bag->velocity[0];
      u2_bag=msg_bag->velocity[1];
      u3_bag=msg_bag->velocity[2];
      u4_bag=msg_bag->velocity[3];
      
      wz_bag = r/4/(l+w)*(-u1_bag+u2_bag+u3_bag-u4_bag);
      
      v_bag[i]=wz_bag; 

      // ROS_INFO("%f \n",v_bag[i]);
      
      fout1<<v_bag[i];
      fout1<<"b\n";
      fout1.close();
      
      i++;
    }
  }

void chatterCallback(const geometry_msgs::TwistStamped::ConstPtr &msg_computed){

        v_comp[i]=msg_computed->twist.angular.z;
      
       ROS_INFO("%f \n",v_comp[i]);
      
      fout2 << v_comp[i];
      fout2 <<"a\n";
      fout2.close();
}


private:
  ros::NodeHandle n;

  ros::Subscriber velocity_bag;
  ros::Subscriber velocity_sub;

  std::ofstream fout1;
  std::ofstream fout2;

  double r;
  double l;
  double w;
   
   float u1_bag; 
   float u2_bag; 
   float u3_bag; 
   float u4_bag; 

  int i;
  float vx_computed;
  float vy_computed;
  float wz_computed;

  float vx_bag;
  float vy_bag;
  float wz_bag;

  float v_bag[N];
  float v_comp[N];
  bool flag1 ;
  bool flag2 ;
  

};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Node4");

  Subscriber Node4;

  ros::spin();

  return 0;
}
