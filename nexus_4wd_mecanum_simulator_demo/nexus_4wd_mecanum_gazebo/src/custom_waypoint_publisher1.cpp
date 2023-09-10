#include <iostream> // Enables command line input and output
 
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Pose2D waypoint; // Goal location ... x, y, and theta 
 
// Ask user for the desired waypoint...Where do you want the robot to move to?
void set_waypoint() {
  cout << "Where do you want the robot to go?" << endl; 
  cout << "Enter waypoint x: ";
  cin >> waypoint.x;
  cout << "Enter waypoint y: ";
  cin >> waypoint.y;
  cout << "Enter waypoint theta (in rad): ";
  cin >> waypoint.theta;
  cout << endl;  
}
 
int main(int argc, char **argv) { 
 
  // Initiate ROS
  ros::init(argc, argv, "custom_waypoint_publisher1");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;
 
  // Publish waypoint to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are not able to be processed quickly enough.
  ros::Publisher waypointPub =
    node.advertise<geometry_msgs::Pose2D>("waypoint", 0);
 
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok()) {
 
    // Ask the user where he wants the robot to go
    set_waypoint();
 
    // Publish the waypoint to the ROS topic
    waypointPub.publish(waypoint);
 
  }
 
  return 0;
}
