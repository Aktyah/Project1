
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "geometry_msgs/TwistStamped.h"
#include "Prj1/wheels.h"
#include "math.h"
#include "std_msgs/Header.h"

class Subscriber {
  public:
          Subscriber() {
            velocity_sub = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel",1000,&Subscriber::chatterCallback,this);
            velocity_pub = n.advertise<Prj1::wheels>("wheels_rpm", 1000);

            n.getParam("/radius", r);
            n.getParam("/lenght", l);
            n.getParam("/width",  w);
            }


 void chatterCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
   
   
  
   v_bx=msg->twist.linear.x;
   v_by=msg->twist.linear.y; 
   w_bz=msg->twist.angular.z; 
 
  // ROS_INFO("v_bx: [%f]", v_bx); 
  // ROS_INFO("v_by: [%f]", v_by); 
  // ROS_INFO("w_bz: [%f]", w_bz); 

   wheels_vel.header.frame_id = msg->header.frame_id;
   wheels_vel.header.stamp.sec = msg->header.stamp.sec;
   wheels_vel.header.stamp.nsec = msg->header.stamp.nsec;
   
   u1=1/r*((-l-w)*w_bz+v_bx-v_by); 
   u2=1/r*((l+w)*w_bz+v_bx+v_by); 
   u3=1/r*((l+w)*w_bz+v_bx-v_by); 
   u4=1/r*((-l-w)*w_bz+v_bx+v_by);  
 
   wheels_vel.rpm_fl=u1*60*5; 
   wheels_vel.rpm_fr=u2*60*5;
   wheels_vel.rpm_rr=u3*60*5;
   wheels_vel.rpm_rl=u4*60*5;

   velocity_pub.publish(wheels_vel) ;

   
    // ROS_INFO("u1: %f", u1); 
    // ROS_INFO("u2: %f", u2); 
    // ROS_INFO("u3: %f", u3);  
    // ROS_INFO("u4: %f", u4); 

  }  
private:
   ros::NodeHandle n;
   ros::Subscriber velocity_sub;
   ros::Publisher  velocity_pub;

   float u1;
   float u2;
   float u3;
   float u4;

   float v_bx;
   float v_by;
   float w_bz;
   
   double r;
   double l;
   double w;   

   std_msgs::Header header1;


  
   Prj1::wheels wheels_vel;
};


int main(int argc, char **argv) {

 ros::init(argc, argv, "Node3");
 
 Subscriber Node3;

  ros::spin();
  return 0;
}
