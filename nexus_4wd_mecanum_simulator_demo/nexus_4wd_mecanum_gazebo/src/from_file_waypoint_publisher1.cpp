#include <iostream> // Enables command line input and output
 #include <fstream>  //To start an ifstream
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "std_msgs/Bool.h"
 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Pose2D waypoint; // Goal location ... x, y, and theta 
std::ifstream infile("/home/johnerzz/Documents/school/thesis/poulakakis/LIP_MPC/vineyard/map/waypoints_for_ros.txt");
bool waypointReached = true;

// Get the desired waypoint from file. Line by Line. Where do you want the robot to move to?
void set_waypoint() {
 
  cout << "Reading pair of coordinates from file." << endl; 
  infile >> waypoint.x >> waypoint.y;  
  cout << waypoint.x << endl;
  cout << waypoint.y << endl;
}

void getGoalReached(const std_msgs::Bool &goalReached) {
  waypointReached = goalReached.data;
}
 
int main(int argc, char **argv) { 
 
  // Initiate ROS
  ros::init(argc, argv, "from_file_waypoint_publisher1");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;
 
  // Publish waypoint to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are not able to be processed quickly enough.
  ros::Publisher waypointPub =
    node.advertise<geometry_msgs::Pose2D>("waypoint", 0);

  //check if previous waypoint has been reached
  //before publishing next waypoint
  ros::Subscriber goalReachedSub =
    node.subscribe("goalReached", 0, getGoalReached);
 
    
  // Specify a frequency that want the while loop below to loop at
  // Keep running the while loop below as long as the ROS Master is active.  
  // If the robot has reached the previous waypoint, set the next waypoint.
  ros::Rate loop_rate(10); 
  while (ros::ok()) {
    ros::spinOnce();
    //cout << waypointReached << endl;
    if (waypointReached)
      {
        set_waypoint();
        // Publish the waypoint to the ROS topic
        waypointPub.publish(waypoint);
      }
    loop_rate.sleep();
  }
 
  return 0;
}
