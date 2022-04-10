#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pub_sub/Num.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "math.h"


class Node1
{
public:
  Node1(){
    ROS_INFO("Node1 working");
    n.getParam("/radius", r);
    n.getParam("/lenght", l);
    n.getParam("/width", w);
    n.getParam("/cpr", cpr);

    ROS_INFO("cpr %d", cpr);


    sub_reader = n.subscribe <sensor_msgs::JointState>("wheel_states", 1000, &Node1::readerCallback, this);
    vel_pub = n.advertise <geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }

  void readerCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  //ROS_INFO("wheel state received");

  //leggo i dati dalla bag
  //tempo delle acquisizioni
  sec = msg->header.stamp.sec;
  nsec = msg->header.stamp.nsec;
  //dati sulleposizioni ruote(tick)
  position_1 = msg->position[0];
  position_2 = msg->position[1];
  position_3 = msg->position[3];
  position_4 = msg->position[2];

  ROS_INFO("position_1: %d", position_1);
  ROS_INFO("position_2: %d", position_2);
  ROS_INFO("position_3: %d", position_3);
  ROS_INFO("position_4: %d", position_4);

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

    delta_t = delta_nsec+ (double) delta_sec;
    ROS_INFO("delta_t: [%f]", delta_t);

    //calcolo dell' angolo
    delta_theta_1 = 2*M_PI/cpr*(position_1-past_position_1);
    delta_theta_2 = 2*M_PI/cpr*(position_2-past_position_2);
    delta_theta_3 = 2*M_PI/cpr*(position_3-past_position_3);
    delta_theta_4 = 2*M_PI/cpr*(position_4-past_position_4);

    ROS_INFO("delta_theta_1: %f", delta_theta_1);
    ROS_INFO("delta_theta_2: %f", delta_theta_2);
    ROS_INFO("delta_theta_3: %f", delta_theta_3);
    ROS_INFO("delta_theta_4: %f", delta_theta_4);

    //velocità angolari delle singole ruote [rad/s]
    u1 = delta_theta_1/delta_t;
    u2 = delta_theta_2/delta_t;
    u3 = delta_theta_3/delta_t;
    u4 = delta_theta_4/delta_t;

    ROS_INFO("-----RPM-----");
    ROS_INFO("u1: %f", u1*60/(2*M_PI));
    ROS_INFO("u2: %f", u2*60/(2*M_PI));
    ROS_INFO("u3: %f", u3*60/(2*M_PI));
    ROS_INFO("u4: %f", u4*60/(2*M_PI));

    //velocità del robot [m/s e rad/s]
    w_bz = r/4/(l+w)*(-u1+u2+u3-u4);
    v_bx = r/4*(u1+u2+u3+u4);
    v_by = r/4*(-u1+u2-u3+u4);


    ROS_INFO("v_bx: %f", v_bx);
    ROS_INFO("v_by: %f", v_by);
    ROS_INFO("w_bz: %f", w_bz);

    //creo messaggio di tipo geometry_msgs/TwistStamped
    geom_msg.header.frame_id = "Robot velocities";
    geom_msg.header.stamp.sec = past_sec;
    geom_msg.header.stamp.nsec = past_nsec;
    geom_msg.twist.linear.x = v_bx;
    geom_msg.twist.linear.y = v_by;
    geom_msg.twist.linear.z = 0;
    geom_msg.twist.angular.x = 0;
    geom_msg.twist.angular.y = 0;
    geom_msg.twist.angular.z = w_bz;

    vel_pub.publish(geom_msg);

    past_position_1 = position_1;
    past_position_2 = position_2;
    past_position_3 = position_3;
    past_position_4 = position_4;
    past_sec = sec;
    past_nsec = nsec;
  }

}

private:
  ros::NodeHandle n;
  ros::Subscriber sub_reader;
  ros::Publisher vel_pub;

  double r;
  double l;
  double w;
  int cpr; //counts per revolution

  bool flag=1;
  int past_sec;
  int past_nsec;
  int past_position_1;
  int past_position_2;
  int past_position_3;
  int past_position_4;

  geometry_msgs::TwistStamped geom_msg;

  int sec;
  int nsec;
  
  int position_1 ;
  int position_2 ;
  int position_3 ;
  int position_4 ;

  double delta_theta_1; 
  double delta_theta_2;
  double delta_theta_3;
  double delta_theta_4;
  
  int delta_sec;
  double delta_nsec;
  double delta_t;

  double u1 ;
  double u2 ;
  double u3 ;
  double u4 ;
  double w_bz ;
  double v_bx ;
  double v_by ;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  Node1 node1;
  ros::spin();
  return 0;
}
