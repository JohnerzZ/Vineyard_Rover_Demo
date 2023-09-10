#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
#include "tf/transform_datatypes.h"
#include "LinearMath/btMatrix3x3.h" //to convert quaternion to roll-pitch-yaw
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include <nav_msgs/Odometry.h> // Needed for accessing Odometry data
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Current x, y, and theta 
geometry_msgs::Pose2D waypointGoal; // Waypoint x, y, and theta (the waypoint)
std_msgs::Float64 yaw;
std_msgs::Float64 speed_abs;
std_msgs::Bool goalReached;
ros::Publisher velocityPub; // Object used for publishing velocity command
ros::Publisher yawPub;
ros::Publisher goalReachedPub;
ros::Publisher speedPub;
 
const double PI = 3.141592654;
// accumulating error for the integral term 
// and previous error for the derivative term (linear velocity)
double Ei_l = 0;
double Ed_l = 0;

// accumulating error for the integral term 
// and previous error for the derivative term (angular velocity)
double Ei_a = 0;
double Ed_a = 0;

 
// The gains K, which are used to calculate the linear velocity
const double Kp_l = 0.2;
const double Ki_l = 0.015;
const double Kd_l = 0.2;
 
// The gains K, which are used to calculate the angular velocity
const double Kp_a = 0.6;
const double Ki_a = 0.01;
const double Kd_a = 0.2;
 
// The distance threshold in meters that will determine when 
// the turtlesim robot successfully reaches the goal.
const double distanceTolerance = 0.17;
 
// The angle threshold in radians that will determine when 
// the turtlesim robot successfully reaches the goal.
const double angleTolerance = 0.1;
 
// This flag determines when the robot needs to either 
// move towards a waypoint or stop.
bool goToWaypoint = false;
 
// Initialized variables and take care of other setup tasks
void setup() {
 
  // We initialize with the default starting coordinate 
  // for the simulator
  waypointGoal.x = 1.5;
  waypointGoal.y = 1.5;
   
  // Initialize the Twist message.
  // Initial linear and angular velocities are 0 m/s and rad/s, respectively.
  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}
 
// Get the distance between the current x,y coordinate and 
// the desired waypoint x,y coordinate.
double getDistanceToWaypoint() {
  return sqrt(pow(waypointGoal.x - current.x, 2) + pow(
    waypointGoal.y - current.y, 2));
}
 
// Get the heading error
// i.e. how many radians does the robot need 
// to turn to head towards the waypoint  
double getHeadingError() {
 
  double deltaX = waypointGoal.x - current.x;
  double deltaY = waypointGoal.y - current.y;
  double waypointHeading = atan2(deltaY, deltaX);
  double headingError = waypointHeading - current.theta;   
  
  //cout << "WaypointHeading: " << waypointHeading << endl;
  //cout << "HeadingError: " << headingError << endl;
   
  // Make sure heading error falls within -PI to PI range
  if (headingError > PI) {
    headingError = headingError - (2 * PI);
  } 
  if (headingError < -PI) {
    headingError = headingError + (2 * PI);
  } 
   
  return headingError;
}
 
// If we haven't yet reached the goal, set the velocity value.
// Otherwise, stop the robot.
void setVelocity() {
 
  double distanceToWaypoint = getDistanceToWaypoint();
  double headingError = getHeadingError();
  double ei_l = 0;
  double ed_l = 0;
  double ei_a = 0;
  double ed_a = 0;
  double dt = 0.1;
 
  // If we are not yet at the waypoint
  if (goToWaypoint == true && (abs(distanceToWaypoint) > distanceTolerance)) {
    //calculate linear velocity pid errors
    ei_l = Ei_l + distanceToWaypoint * dt;
    ed_l = (distanceToWaypoint - Ed_l) / dt;
    cout << ed_l << endl;
    //calculate angular velocity pid errors
    ei_a = Ei_a + headingError * dt;
    ed_a = (headingError - Ed_a) / dt;

    // If the robot's heading is off, fix it.
    if (abs(headingError) > angleTolerance) {
      //velCommand.linear.x = 0.0;
      velCommand.angular.z = Kp_a  * headingError + Ki_a * ei_a + Kd_a * ed_a;
      Ei_a = ei_a;
      Ed_a = headingError;

    }
    // Just fix the distance gap between current pose and waypoint.
    // The magnitude of the robot's velocity is directly
    // proportional to the distance the robot has from the 
    // goal.
    velCommand.linear.x = Kp_l * distanceToWaypoint + Ki_l * ei_l + Kd_l * ed_l;
    //velCommand.angular.z = 0.0;  
    Ei_l =  ei_l;
    Ed_l = distanceToWaypoint;  
    


  }
  else {
    //cout << "Goal has been reached!" << endl << endl;
    velCommand.linear.x -= 0.5*velCommand.linear.x;
    velCommand.angular.z = 0.0; 
    goToWaypoint = false;
  }
}
 
// This callback function updates the current position and 
// orientation of the robot. 
void updatePose(const nav_msgs::Odometry::ConstPtr &currentPose) {
  current.x = currentPose->pose.pose.position.x;
  current.y = currentPose->pose.pose.position.y;
  tf::Quaternion q(
    currentPose->pose.pose.orientation.x,
    currentPose->pose.pose.orientation.y,
    currentPose->pose.pose.orientation.z,
    currentPose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);			//convert quaternions to roll, pitch, yaw.
    current.theta = yaw;
}
 
// This callback function updates the desired waypoint when a waypoint
// message is published to the /waypoint topic
void updateWaypoint(const geometry_msgs::Pose2D &waypointPose) {
  waypointGoal.x = waypointPose.x;
  waypointGoal.y = waypointPose.y;
  goToWaypoint = true;  
}
 
int main(int argc, char **argv) {
 
  setup();  
 
  // Initiate ROS
  ros::init(argc, argv, "go_to_goal_x_y");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;
 
  // Subscribe to the robot's pose
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new pose is received, update the robot's pose.
  ros::Subscriber currentPoseSub =
    node.subscribe("/odom", 0, updatePose);
     
  // Subscribe to the user's desired waypoint
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new waypoint is received, update the robot's 
  // desired waypoint.
  // The tcpNoDelay is to reduce latency between nodes and to make sure we are
  // not missing any critical waypoint messages.
  ros::Subscriber waypointPoseSub =
    node.subscribe("waypoint", 0, updateWaypoint, 
    ros::TransportHints().tcpNoDelay());
 
  // Publish velocity commands to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  velocityPub =
    node.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

  speedPub = 
    node.advertise<std_msgs::Float64>("/speed_abs", 0);
    
  yawPub = 
    node.advertise<std_msgs::Float64>("/true_yaw", 0);

  goalReachedPub = 
    node.advertise<std_msgs::Bool>("/goalReached", 0);
 
  // Specify a frequency that want the while loop below to loop at
  // In this case, we want to loop 10 cycles per second
  ros::Rate loop_rate(10); 
 
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok()) {
 
    // Here is where we call the callbacks that need to be called.
    ros::spinOnce();
 
    // After we call the callback function to update the robot's pose, we 
    // set the velocity values for the robot.
    setVelocity();
 
    // Publish the velocity command to the ROS topic
    velocityPub.publish(velCommand);
    
    // Publish yaw to /true_yaw
    yaw.data = current.theta;
    yawPub.publish(yaw);

    //publish speed absolute value to /speed_abs
    speed_abs.data = sqrt(pow(velCommand.linear.x, 2) + pow(velCommand.linear.y, 2));
    speedPub.publish(speed_abs);


    //publish goal has been reached boolean
    goalReached.data = !goToWaypoint;
    goalReachedPub.publish(goalReached);
 
    // Print the output to the console
    /*cout << "Current (x,y) = " << "(" << current.x << "," << current.y << ")"
         << endl
         << "Current theta = " << "(" << current.theta << ")"
         << endl
         << "Waypoint (x,y) = " << "(" << waypointGoal.x << ","
         << waypointGoal.y << ")"
         << endl
         << "Distance to Waypoint = " << getDistanceToWaypoint() << " m"
         << endl
         << "Linear Velocity (x) = " << velCommand.linear.x << " m/s"
         << " Linear Velocity (y) = " << velCommand.linear.y << " m/s"
         << endl
         << endl << endl;*/
 
    // Sleep as long as we need to to make sure that we have a frequency of
    // 10Hz
    loop_rate.sleep();
  }
 
  return 0;
}

