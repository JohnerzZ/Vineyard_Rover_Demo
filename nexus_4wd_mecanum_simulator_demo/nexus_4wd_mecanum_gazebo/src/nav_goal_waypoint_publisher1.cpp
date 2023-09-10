#include <iostream> // Enables command line input and output
 
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "geometry_msgs/PoseStamped.h"  //messages received from 2D_nav_goal are PoseStamped
#include "tf/transform_datatypes.h"

 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Pose2D waypoint; // Goal location ... x, y, and theta 

// Set initial waypoint based on robot's initial position.

void setup() {
  waypoint.x = 1.5;
  waypoint.y = 1.5;
  waypoint.theta = 0;
}

// set the waypoint given by 2D_nav_goal in rviz.

void set_waypoint(const geometry_msgs::PoseStamped::ConstPtr &waypointPose) {
  cout << "got in" << '\n';
  waypoint.x = waypointPose->pose.position.x;
  waypoint.y = waypointPose->pose.position.y;
  tf::Quaternion q(
    waypointPose->pose.orientation.x,
    waypointPose->pose.orientation.y,
    waypointPose->pose.orientation.z,
    waypointPose->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);     //convert quaternions to roll, pitch, yaw.
  waypoint.theta = yaw;
  cout << "Given waypoint x: ";
  cout << waypoint.x << '\n';
  cout << "Given waypoint y: ";
  cout << waypoint.y << '\n'; 
  cout << "Given waypoint theta: ";
  cout << waypoint.theta << '\n'; 
}

 

int main(int argc, char **argv) {

   
  setup();  
   
  // Initiate ROS
  ros::init(argc, argv, "nav_goal_waypoint_publisher1");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;
  
  ros::Subscriber navGoalSub =
  node.subscribe("move_base_simple/goal", 0, set_waypoint);
 
  // Publish waypoint to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are not able to be processed quickly enough.
  ros::Publisher waypointPub =
    node.advertise<geometry_msgs::Pose2D>("waypoint", 0);
    
  ros::Rate loop_rate(10);
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok()) {
  
    ros::spinOnce();   
    // Publish the waypoint to the ROS topic
    waypointPub.publish(waypoint);
    loop_rate.sleep();
  }
 
  return 0;
}
