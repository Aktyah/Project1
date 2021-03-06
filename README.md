# Project1

Project1 ROS

Desription of all the work done by:

******** El Hanafi Ahmed

******** Alessandro Cupo

******** Pasqualotto Marco 
***
### Content of the zip

- cfg: file for dynamic reconfegure of integration mode
- launch: launch file called Prj1.launch
- msg: 3 custom messages: v_bag and v_computed were used in the calibration process, while wheels was a request for the project
- lib and include: these files were used in order to create a library for Node2
- src: it contains the four nodes run by the launchfile. 

1) Node1: it subscribes to /wheels_states, advertising a message of type sensor_msgs::JointState, reads the tick provided by the encoder and computes the velocities of robot considering the dynamics of an omnidirectional wheeled mobile robot. It publishes the velocities on the topic /cmd_vel advertising a message of type geometry_msgs::TwistStamped.


2) Node2: it's main job is integrating the velocities published from Node1 in /cmd_vel, using either Euler or Runge-Kutta method, it then publishes the odometry computed in the topic /odom (the message is of tipe "nav_msgs::Odometry").
When the node starts it reads the first position configurable from the launch file using the function "setFirstPose" (we set the position to [0 0 0] [0 0 0 1]).
Whenever a new bag is run the node will automatically reset the odom TF to the first pose of the bag (more to that in the tf chapter) and then the robot to the origin of odom TF. This is done thanks to the subscription called "first_pose_sub" that reads the "geometry_msgs::PoseStamped" message from the topic /robot/pose.
The node also defines a service, "new_pose", that allows the user to change the pose of the robot (it moves the odom TF to the given position, then the robot to it's origin).
There is also defined a dynamic reconfigure that allows the selection of the integration method.
Finally the node is able to brodcast the TF tree of the three reference frames world, odom and base_link (the tree is reported in the TF tree chapter).
Since this note uses lots of functions, it was decided to use a library to store them.

3) Node3: subscribes to the topic published by Node1 /cmd_vel, while publishing on a new topic /wheels_rpm. This topic publish, as requested, into a custom msg wheels. This message is formed by Header and the four RPMs of the wheels. What the node do is: it takes the value of the velocites (Vx,Vy,Wz) published on cmd_vel and then it applies the inverse of the kinematic relation used in the Node1. So it converts the three velocities in the four velocities of the wheels (The velocity published on wheels_rpm are in RPM). Plus it copies the Header of cmd_vel, which in turn is the Header given by the bag, and print it in the new msg wheels.

4) Node4: subscribes to the two topics /cmd_vel and /wheel_states. From those two topics the subscribers read and computes the appropriate velocity, either Vx or Wz. Which velocity is evaluated depend on what parameter is given to the node when calling it from the terminal, further explanation will be given later on. Then those two values will be pubblished into two different custom messages: "v_bag" and "v_computed" both containing a type float. 

- srv: it contains the service in charge of the dynamic reconfigure of the pose of the robot.

- regression: In this folder it is possible to find the data and the Matlab script used to perform parameter optimization through Linear Regression and OLS equation. In particular, there are two folders: V_x and W_z. They refer to which velocity is saved inside the txt files. In each folder there are two txt files "v_bag" and "v_computed", the first contains all the velocities (either Vx or Wz depending on the folder) obatined from the bag files, meanwhile the second contain the same type of velocity but computed from us (those are the value of velocity published on the topic /cmd_vel). Explanation on how we implemented the regression will be given later on.
Note: We gave the .txt files to show what are the actual values we used for optimization using the bag that were given to us. Altough, as long as the data are saved using the procedure later explained, the Node4 and Matlab Script would perform the linear regression regardless the particular bag.

***
### Node Structure

![rosgraph1](https://user-images.githubusercontent.com/89984587/167246375-444e91f5-06f4-4803-877a-664e29af8367.png)


***

### Ros Parameters
In the launch fie we specify the main parameters of the robot:
1) radius of the wheels: 0.07 m (we calibrate it and set at 0.077 m)
2) length of the robot: 0.20 m (we leave this as it is)
3) width of the robot: 0.169 m (we calibrate it and set at 0.1545 m; we decide to calibrate only the width because we can only estimate the sum length+width, so we change only the width parameter; we could have changed also the leg or even both, bacause the robot dynamics is influenced by width+length
4) cpr, counts per revolution: 42 (after calibrating the other parameter we were fulfilled with the results, so we leave this as it is. We thougth that this parameter is given by the productor of the encoder and it cannot be off)
5) FirstPose.. this set of parameters sets the initial pose of the robot, if a bag is played our program sets the robot in the position given by the bag

### TF tree:
![frames](https://user-images.githubusercontent.com/89984587/167246068-7d34fe49-a8a9-4d9f-af66-2e3ac8182a07.png)

The tree contains 3 reference systems:
 1) world is the main (fixed tp the ground) reference system that is generated from the bags
 2) odom is the reference system on which odometry is computed. It's allways reset in the first position of the robot in the bag (this happenes whenever a new bag is played).
 3) base_link is the robot's odometry local reference system hence it allways follows the odometry. In this project it was simply a requirement, but in general it could be used to trak components of then robot such as cameras, wheels ecc (whatever sould be too hard to describe in a global reference system).
 ***
 
### Custom Messages

There are a total of three Custom Messages implemented:
v_bag 
v_computed
wheels

The first two are used in order to calibrate the parameters. Both of this are formed by a message of type float64. In the case of v_bag the variable is a float64 of name "vbag". For v_computed the variable is still a float64 named "vcomputed".
Lastly, the wheels msg is the one asked from the project. It's structure is formed by: an Header header, four float64 variables where the velocity in Round Per Minute of the wheels are saved. The names, as stated in the pdf are "rpm_fl" "rpm_fr" "rpm_rr" "rpm_rl".

### How to use the Node
All of the main Nodes are ment to be started with a launchfile (roslaunch Prj1 Prj1.launch), then all of the    main tasks required from the exerise will be computed except for the calibration node,Node4, that must be started manually (since it's not meant to be run often).
The package, more specificly Node2, gives the possibility to change the integration method between Euler (that is set by default at the beginning) and Runge-Kutta (rosrun rqt_reconfigure rqt_reconfigure, this hsould launch a gui program that allows the change of the integration mehtod).
It's also possible to reset the odom reference system to any given position, moving consequiently the odometry (rosservice new_pose x y theta, where x y theta are supposed to be new coordinates).


- Node4 and Matlab script: 

The Node is not directly open from launch file, because calibration should only be done once. Therefore, from terminal we can call the node :" rosrun Prj1 Node4 0" or "rosrun Prj1 Node4 1". The input 0/1 sent to the node depends on what we wish to calibrate. If R (radius) then use 0, if l and w (length and width) use 1. Afterwards, if we want R we should run the bag1, and calibration is performed using the first part of the bag where we only have the component Vx. For l+w run bag2 where we only have value of Wz. At this point we created a .txt file containing  a copy of the data published onto the two topics velocity_bag and velocity_computed. This has been done by using a modified version of echo: " rostopic echo /velocity_bag >> v_bag.txt " and      " rostopic echo /velocity_computed >> v_computed.txt " (those two commands are called before the topic start publishing so to not loose any value of the velocity). In our case what this echo returns from our bags has been reported in the .txt files. Then the matlab scripts simply open the .txt files, modify the format of the data to extract the needed numerical value from the string and later perform the linear regression by appling the OLS. What matlab gives back is the weight w and a scatter plot of the two velocity and the line.
Note: The matlab script open two files named "v_bag.txt" and "v_computed.txt". We kept the script outside the two folders where the two .txt files for Vx and Wz are saved. This to avoid to copy and paste two identical script. Based on what parameter we wish to calibrate (R or l+w) we should move the script inside the corrispondent folder (Vx or Wz).
 
***
 ### Miscellaneous
 **********************************
- Idea behind linear regression and parameter calibration.

 For parameter calibration we used the following idea. Firstly we implemented all the parameters inside the launch file. All nodes get their parameter through launch file. In this way if a "manual" calibration of the parameter is needed it can be done by just changing their value from the launch file, either from terminal or manually, without the need to recompile the code.
Morevore, we noticed that the parameters that provided the best correction of our odometry where: R,l,w. Therefore we tried an optimization of those parameters using a linear regression. This operation is done through the Node4 and a matlab script that, given the data, apply the OLS formula.
What we did was: for parameter R collect the two different velocity, the one from the bag which we called "v_bag" and the one computed by us "v_computed", only focusing the Vx component. Therefore we used the first part of the bag1 where the robot moved only in the x direction without rotating. The coefficient obtained from the linear regression was w=1.096. Because Vx is proportional to the radius R we simply multiplied and used Ropt=R*w. For l and w we used the Wz component of the velocity. Firstly we set as value of R the optimal one previously computed and, in the same way as before, we collect the two velocities "v_bag" and "v_computed" and then applied OLS. In this case the bag2 was used where only rotation are given as input to the robot. The results was a value of w=1.0403 . Because Wz is proportional to 1/(l+w), to obtain the optimal sum of this two values we divided: (l+w)opt=(l+w)/w. Then the difference between the two l+w was subtracted to w. As stated before we chose to vary parameter w, but as long as the sum of the two paramters l+w is equal to the optimal one computed how we change the parameter should be irreleveant. We could have changed only l or both.

