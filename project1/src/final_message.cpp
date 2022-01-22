#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "project1/MethodMessage.h"
#include <sstream>

//This node subscribes to two topics: "robot_odom", containing the computed odometry of the robot,
//and "method", containing the integration method defined by dynamic reconfigure in the node "ERK".
//Then, the node publishes on the topic "final_chatter" a custom message, called "MethodMessage"
//where the previous two are merged (TASK 4)

class pub_sub {

public:
  project1::MethodMessage out;

private:
  ros::NodeHandle n; 

  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Publisher pub; 
  ros::Timer timer1;
  
public:
  pub_sub(){
    sub = n.subscribe("robot_odom", 1, &pub_sub::callback_m1, this);
    sub2 = n.subscribe("method", 1, &pub_sub::callback_m2, this);
    pub = n.advertise<project1::MethodMessage>("final_chatter", 1);
    timer1 = n.createTimer(ros::Duration(0.005), &pub_sub::callback_t, this); //0.005 otherwise there is a delay in the method change
  }

  void callback_m1(const nav_msgs::Odometry::ConstPtr& msg){
    out.header.frame_id = msg->header.frame_id;
    out.child_frame_id = msg->child_frame_id;
    out.pose = msg->pose;
    out.twist = msg->twist;
  }

  void callback_m2(const std_msgs::String::ConstPtr& msg1){
    std::stringstream ss;
    ss << *msg1;
    out.method = ss.str();
  }

  void callback_t(const ros::TimerEvent&) {  
    pub.publish(out);
    ROS_INFO("Publishing message containing odometry and integration method");
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "final_message");
  
  pub_sub pub_sub;
  
  ros::spin();
  
  return 0;
}
