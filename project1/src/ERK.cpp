#include "ros/ros.h"
#include "project1/MotorSpeed.h"
#include "project1/pre_speed.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>
#include "tf/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>
#include "project1/reinit_pose.h"
#include "project1/reselect_pose.h"

//This node subscribes to the topic "lin_ang" and publishes two messages on two different topics.
//The topic "robot_odom" contains the odometry computed by using either Euler or Runge-Kutta method.
//The node also contains the dynamic reconfigure to select in real-time the integration method.
//The topic "method" contains the selected integration method. The initial pose for the odometry 
//is taken by a parameter defined in the launch file, that can be manually changed from there.
//Moreover, it includes 2 services, reinit_pose and reselect_pose. While reinit_pose initialize the pose
//to 0,0,0 (set position to 0, 0 and the yaw angle theta to 0 deg, too), reselect_pose allows you to redefine 
//a pose for your robot using the coordinate x and y for the position and theta as the yaw angle orientation.


std_msgs::Int16 int_enum;

double xp;
double yp;
double theta;

class odometry {

public:
  nav_msgs::Odometry out1;
  std_msgs::String out2;
  double v;
  double ome;
  ros::NodeHandle nh;
  
private:
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Publisher pub1; 
  ros::Publisher pub2; 
  ros::Timer timer1;
  float Ts=0.001;
  
  tf2::Quaternion quat;
  geometry_msgs::Quaternion quatmsg;
  
public:
  odometry(){
    sub1 = nh.subscribe("lin_ang", 1, &odometry::callback_m1, this);
    pub1 = nh.advertise<nav_msgs::Odometry>("robot_odom", 1);
    pub2 = nh.advertise<std_msgs::String>("method", 1);   
    timer1 = nh.createTimer(ros::Duration(0.001), &odometry::callback_t, this);
  }

  void callback_m1(const geometry_msgs::TwistStamped::ConstPtr& msg){
    v = msg->twist.linear.x;
    ome = msg->twist.angular.z;

    if(int_enum.data==0){
      theta = theta + ome * Ts;
      xp = xp + v * Ts * cos(theta);
      yp = yp + v * Ts * sin(theta);

      std::stringstream ss;
    	ss << "euler";
    	out2.data = ss.str();
    }

    if(int_enum.data==1){
      theta = theta + ome * Ts;
      xp = xp + v * Ts * cos(theta + ome * Ts / 2);
      yp = yp + v * Ts * sin(theta + ome * Ts / 2);

      std::stringstream ss;
    	ss << "rk";
    	out2.data = ss.str();
    }

    geometry_msgs::Quaternion quatmsg = tf::createQuaternionMsgFromYaw(theta);

    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = xp;
    odom_trans.transform.translation.y = yp;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = quatmsg;

    odom_broadcaster.sendTransform(odom_trans);

    out1.header.frame_id = "odom";

    out1.pose.pose.position.x = xp;
    out1.pose.pose.position.y = yp;
    out1.pose.pose.position.z = 0;
    out1.pose.pose.orientation = quatmsg;

    out1.child_frame_id = "base_link";

    out1.twist.twist.linear.x = v;
    out1.twist.twist.linear.y = 0;
    out1.twist.twist.linear.z = 0;
    out1.twist.twist.angular.x = 0;
    out1.twist.twist.angular.y = 0;
    out1.twist.twist.angular.z = ome;
  }

  void callback_t(const ros::TimerEvent&) {
    pub1.publish(out1);
    pub2.publish(out2);
  }
};

void dynRec(project1::parametersConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", config.type);
  int_enum.data=config.type;
  ROS_INFO ("%d",level);
}

bool reinit_pose(project1::reinit_pose::Request  &req,
         project1::reinit_pose::Response &res){
  res.x0 = 0;
  res.y0 = 0;
  res.th0 = 0;
  xp = res.x0;
  yp = res.x0;
  theta = res.th0;
  return true;
}

bool reselect_pose(project1::reselect_pose::Request &req,
         project1::reselect_pose::Response &res){
  res.x0 = req.xD;
  res.y0 = req.yD;
  res.th0 = req.thD;
  xp = res.x0;
  yp = res.y0;
  theta = res.th0;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ERK");
  ros::NodeHandle n; 

  std::vector<double> initialPosition;
  n.getParam("initialPosition", initialPosition);
  xp=initialPosition[0];
  yp=initialPosition[1];
  theta=initialPosition[2];

  ros::ServiceServer service = n.advertiseService("reinit_pose", reinit_pose);
  ros::ServiceServer service2 = n.advertiseService("reselect_pose", reselect_pose);

  dynamic_reconfigure::Server<project1::parametersConfig> server;
  dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
  f = boost::bind(&dynRec, _1, _2);
  server.setCallback(f);

  odometry my_pub_sub;
  
  ros::spin();
  
  return 0;
}
