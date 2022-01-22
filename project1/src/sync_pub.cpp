#include "ros/ros.h"
#include "project1/MotorSpeed.h"
#include "project1/pre_speed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/Odometry.h"

//This node subscribes to the topics "motor_speed_fl", "motor_speed_rr" and "scout_odom",
//synchronizes the three messages and publishes on the custom message "pre_speed" the velocities 
//of a left wheel, a right wheel, and the estimation of linear and angular velocities given by 
//the in-built odometry. The speed of the two wheels on the same side are considered equal.

class sync_pub
{
    private:
        ros::NodeHandle n;
        message_filters::Subscriber<project1::MotorSpeed> sub1;
        message_filters::Subscriber<project1::MotorSpeed> sub2;
        message_filters::Subscriber<nav_msgs::Odometry> sub3;
        message_filters::Subscriber<nav_msgs::Odometry> sub4;
        ros::Publisher pub;
        project1::pre_speed msg_out;
        typedef message_filters::sync_policies::ApproximateTime<project1::MotorSpeed, project1::MotorSpeed, nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync; 
    
    public:   
        sync_pub():sync(MySyncPolicy(10), sub1, sub2, sub3, sub4)
        {
            sub1.subscribe(n, "motor_speed_fl", 1);
            sub2.subscribe(n, "motor_speed_rr", 1);
            sub3.subscribe(n, "scout_odom", 1);
            sub4.subscribe(n, "scout_odom", 1);
            pub=n.advertise<project1::pre_speed>("pre_speed", 1);
            sync.registerCallback(boost::bind(&sync_pub::callback, this, _1, _2, _3, _4));
        }

    void callback(const project1::MotorSpeed::ConstPtr& msg1,
                  const project1::MotorSpeed::ConstPtr& msg2,
                  const nav_msgs::Odometry::ConstPtr& msg3,
                  const nav_msgs::Odometry::ConstPtr& msg4){

        ROS_INFO ("Synchronized speeds: left %f and right %f \nlinear %f \nangular %f", 
                    msg1->rpm, msg2->rpm, msg3->twist.twist.linear.x, msg4->twist.twist.angular.z);
        msg_out.FL = msg1->rpm;
        msg_out.RR = msg2->rpm;
        msg_out.lin = msg3->twist.twist.linear.x;
        msg_out.ang = msg4->twist.twist.angular.z;
        pub.publish(msg_out);
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "sync_pub");
    sync_pub sync_pub;
    ros::spin();
    return 0;
}
