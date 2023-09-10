#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
#include "tf/transform_datatypes.h" //to convert quaternion to roll-pitch-yaw
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include <nav_msgs/Odometry.h> // Needed for accessing Odometry data
#include "std_msgs/Float64.h"
 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Current x, y, and theta 
geometry_msgs::Pose2D waypointGoal; // Waypoint x, y, and theta (the waypoint)
std_msgs::Float64 yaw;
ros::Publisher velocityPub; // Object used for publishing velocity command
ros::Publisher yawPub;
 
const double PI = 3.141592654;
 
// The gain K, which is used to calculate the linear velocity
const double K_l = 0.5;
 
// The gain K, which is used to calculate the angular velocity
const double K_a = -0.5;
 
// The distance threshold in meters that will determine when 
// the turtlesim robot successfully reaches the goal.
const double distanceTolerance = 0.05;
 
// The angle threshold in radians that will determine when 
// the turtlesim robot successfully reaches the goal.
const double angleTolerance = 0.3;
 
// This flag determines when the robot needs to either 
// move towards a waypoint or stop.
bool goToWaypoint = true;

// This flag is used when the robot reaches its waypoint.
// It determines if the robot needs to either adjust its
// final yaw or stop.
bool adjustFinalYaw = true;
 
// Initialized variables and take care of other setup tasks
void setup() {
 
  // We initialize with the default starting coordinate 
  // for the simulator
  waypointGoal.x = 1.5;
  waypointGoal.y = 1.5;
  waypointGoal.theta = 0;
   
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
  
  cout << "WaypointHeading: " << waypointHeading << endl;
  cout << "HeadingError: " << headingError << endl;
   
  // Make sure heading error falls within -PI to PI range
  if (headingError > PI) {
    headingError = headingError - (2 * PI);
  } 
  if (headingError < -PI) {
    headingError = headingError + (2 * PI);
  } 
   
  return headingError;
}


double getFinalHeadingError() {
 
  double waypointHeading = waypointGoal.theta;  //For our final adjustment we get our waypointHeading from navGoal.
  double headingError = waypointHeading - current.theta;   
  
  cout << "WaypointHeading: " << waypointHeading << endl;
  cout << "HeadingError: " << headingError << endl;
   
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
 
  // If we are not yet at the waypoint
  if (goToWaypoint == true && (abs(distanceToWaypoint) > distanceTolerance)) {
    
    // If the robot's heading is off, fix it.
    if (abs(headingError) > angleTolerance) {
      velCommand.linear.x = 0.0;
      velCommand.angular.z = - K_a  * headingError;
    }
    // Just fix the distance gap between current pose and waypoint.
    // The magnitude of the robot's velocity is directly
    // proportional to the distance the robot has from the 
    // goal.
    else {
      velCommand.linear.x = K_l * distanceToWaypoint;
      velCommand.angular.z = 0.0;    
    }
  }
  else {
    cout << "Goal has been reached!" << endl << endl;
    velCommand.linear.x = 0.0;
    velCommand.angular.z = 0.0; 
    goToWaypoint = false;
  }
}

void setFinalYaw() { 
  // If the robot has reached the waypoint
  if (goToWaypoint == false && adjustFinalYaw == true) {

    double finalHeadingError = getFinalHeadingError();
    cout << "checking for yaw adjustment." << endl << endl;

    // If the robot's final heading is off, fix it.
    if (abs(finalHeadingError) > angleTolerance) {
      velCommand.linear.x = 0.0;
      velCommand.angular.z = - K_a  * finalHeadingError;
    }

    else {
      adjustFinalYaw = false;
      cout << "Final yaw has been adjusted!" << endl << endl;

    }
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
    cout << "!!!!!RPY!!!!!" << endl;
    cout << roll << ' ' << pitch << ' ' << yaw << endl << endl;
    current.theta = yaw;
}
 
// This callback function updates the desired waypoint when a waypoint
// message is published to the /waypoint topic
void updateWaypoint(const geometry_msgs::Pose2D &waypointPose) {
  waypointGoal.x = waypointPose.x;
  waypointGoal.y = waypointPose.y;
  waypointGoal.theta = waypointPose.theta;
  goToWaypoint = true;  
  adjustFinalYaw = true;
}
 
int main(int argc, char **argv) {
 
  setup();  
 
  // Initiate ROS
  ros::init(argc, argv, "go_to_goal_x_y_yaw");
     
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
    
  yawPub = 
    node.advertise<std_msgs::Float64>("/true_yaw", 0);
 
  // Specify a frequency that want the while loop below to loop at
  // In this case, we want to loop 10 cycles per second
  ros::Rate loop_rate(8); 
 
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok()) {
 
    // Here is where we call the callbacks that need to be called.
    ros::spinOnce();
 
    // After we call the callback function to update the robot's pose, we 
    // set the velocity values for the robot.
    setVelocity();
    // We then check if we need to adjust its final yaw.
    setFinalYaw();
 
    // Publish the velocity command to the ROS topic
    velocityPub.publish(velCommand);
    
    // Publish yaw to /true_yaw
    yaw.data = current.theta;
    yawPub.publish(yaw);
 
    // Print the output to the console
    cout << "Current (x,y) = " << "(" << current.x << "," << current.y << ")"
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
         << endl << endl;
 
    // Sleep as long as we need to to make sure that we have a frequency of
    // 10Hz
    loop_rate.sleep();
  }
 
  return 0;
}
