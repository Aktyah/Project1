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
#include "std_msgs/Float64.h"
#include <dynamic_reconfigure/server.h>
#include "Prj1/parametersConfig.h"
#include "Prj1/pose.h"

#include <cmath>
#include <iostream>

#include "Node2_class/Node2_class.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Node2");
    Node2 node;
    node.run_node();

    return 0;
}