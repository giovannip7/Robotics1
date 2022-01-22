#include "ros/ros.h"
#include "project1/MotorSpeed.h"
#include "project1/pre_speed.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"

//This node subscribes to the topic "pre_speed", created in "sync_pub", and publishes on the topic
//"lin_ang" a message "TwistStamped" with the linear and angular velocities of the robot. These are
//obtained by computations in which the baseline and the gear ratio are obtained by hand.

double CONST_ang = 1 / 1.0383 * 0.1575 * 0.02614 * 6.28 / 60;

class pub_sub {

public:
  geometry_msgs::TwistStamped message1;

private:
  ros::NodeHandle n; 

  ros::Subscriber sub;
  ros::Publisher pub; 
  ros::Timer timer1;
  
public:
  pub_sub(){
    sub = n.subscribe("pre_speed", 1, &pub_sub::callback_m1, this);
    pub = n.advertise<geometry_msgs::TwistStamped>("lin_ang", 1);
    timer1 = n.createTimer(ros::Duration(0.001), &pub_sub::callback_t, this);
}

  void callback_m1(const project1::pre_speed::ConstPtr& msg){
    message1.twist.linear.x= ( - msg->FL + msg->RR ) / 2 * 0.1575 * 0.02614 * 6.28 / 60;
    message1.twist.angular.z= ( msg->FL + msg->RR ) * CONST_ang;
  }

  void callback_t(const ros::TimerEvent&) {
    pub.publish(message1);
    ROS_INFO("Publishing estimated linear and angular speeds: \nLinear:%f \nAngular:%f", 
              message1.twist.linear.x, message1.twist.angular.z);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_and_publish");
  
  pub_sub pub_sub;
  
  ros::spin();
  
  return 0;
}
