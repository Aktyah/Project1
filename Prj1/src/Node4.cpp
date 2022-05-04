
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
#include "Prj1/v_computed.h"
#include "Prj1/v_bag.h"
#include "Prj1/wheels.h"

#define N 100000

class Subscriber
{
public:
  Subscriber()
  {
    velocity_bag = n.subscribe<sensor_msgs::JointState>("wheel_states", 1000, &Subscriber::Callback,this);
    velocity_sub = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1000, &Subscriber::chatterCallback,this);
    v_sub = n.advertise<Prj1::v_computed>("velocity_computed", 1000);
    v_bag = n.advertise<Prj1::v_bag>("velocity_bag", 1000);

    n.getParam("/radius", r);
    n.getParam("/lenght", l);
    n.getParam("/width",  w);
  
    i=0;
    flag1 = 1;
    flag2 = 1;
  }

  void Callback(const sensor_msgs::JointState::ConstPtr& msg_bag)
  {
    // if(flag1 == 1 && flag2 == 1) {
    //   flag1 = 0;
    //    } 
    // else if(flag1 == 0 && flag2 == 1){
    //   flag2 = 0;
    //    } 
    //  else {
      u1_bag=msg_bag->velocity[0]; // fl
      u2_bag=msg_bag->velocity[1]; // fr
      u4_bag=msg_bag->velocity[2]; // rl
      u3_bag=msg_bag->velocity[3]; // rr
      
      vvb.vbag = r/4/(l+w)*(-u1_bag+u2_bag+u3_bag-u4_bag); 
      //vvb.vbag = r/4*(u1_bag+u2_bag+u3_bag+u4_bag);
      v_bag.publish(vvb);

    
  }

void chatterCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
        
        //vvc.vcomputed= msg->twist.linear.x;
        vvc.vcomputed= msg->twist.angular.z;

        vvc.vcomputed=vvc.vcomputed*60*5;
        //vvc.vcomputed = r/4/(l+w)*(-u1_computed+u2_computed+u3_computed-u4_computed); w_z
        
        // vvc.vcomputed = r/4*(u1_computed+u2_computed+u3_computed+u4_computed);

        v_sub.publish(vvc);
         
 }


private:
  ros::NodeHandle n;

  ros::Subscriber velocity_bag;
  ros::Subscriber velocity_sub;

  ros::Publisher  v_sub;
  ros::Publisher  v_bag;

  Prj1::v_computed  vvc;
  Prj1::v_bag       vvb;
  Prj1::wheels      vvw;

  double r;
  double l;
  double w;
   
   float u1_bag; 
   float u2_bag; 
   float u3_bag; 
   float u4_bag; 

   float u1_computed; 
   float u2_computed; 
   float u3_computed; 
   float u4_computed; 

  int i;
  float vx_computed;
  float vy_computed;
  float wz_computed;

  float vx_bag;
  float vy_bag;
  float wz_bag;

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
