# Project1

Project1 ROS

Desription of all the work done by:

10865239 Pasqualotto Marco 

10580177 El Hanafi Ahmed

10864444 Alessandro Cupo

### Content of the zip

- cfg: file for dynamic reconfegure of integration mode
- launch: launch file called Prj.launch******
- msg: 3 custom messages: v_bag and v_computed were used in the calibration process, while wheels was a request for the project
- lib and include: these files were used in order to create a library for Node2
- src: it contains the four nodes run by the launchfile. 
1) Node1: it subscribes to /wheels_states, advertising a message of type sensor_msgs::JointState, reads the tick provided by the encoder and computes the velocities of robot considering the dynamics on an omnidirectional wheeled mobile robot. It publishes the velocities on the topic /cmd_vel advertising a message of type geometry_msgs::TwistStamped.
2) Node2: it's main job is integrating the velocities published from Node1 in /cmd_vel, using either Euler or Runge-Kutta method, it then publishes the odometry computed in the topic /odom (the message is of tipe "nav_msgs::Odometry".
When the node starts it reads the first position configurable from the launch file using the function "setFirstPose" (we set teh position to [0 0 0] [0 0 0 1]).
Whenever a new bag is run the node will automatically reset the odom TF to the first pose of the bag (more to that in the tf chapter) and then the robot to the origin of odom TF. This is done thanks to the subscription called "first_pose_sub" that reads the "geometry_msgs::PoseStamped" message from the topic /robot/pose.
The node also defines a service, "new_pose", that allows the user to change the pose of the robot (it moves the odom TF to the given position, then the robot to it's origin).
There is also defined a dynamic reconfigure that allows the selection of the integration method.
Finally the node is able to brodcast the TF tree of the three reference frames world, odom and base_link (the tree is reported in the TF tree chapter).
3) [ALESSANDRO]
4) [ALESSANDRO]
- srv: it contains the service in charge of the dynamic reconfigure of the pose of the robot.
- regression (outs: [ALESSANDRO]



