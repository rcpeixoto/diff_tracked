#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <math.h>
#include <stdio.h>

#define WHEEL_DISTANCE 0.23
#define TICK_DISTANCE 0.0149253

int left_odom, old_left_odom;
int right_odom, old_right_odom;
double x = 0, y = 0, th = 0;

void left_callback(const std_msgs::Int16::ConstPtr& msg){
    left_odom = msg->data;
}


void right_callback(const std_msgs::Int16::ConstPtr& msg){
    right_odom = msg->data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "diff_robot_odometry");
    tf::TransformBroadcaster odom_broadcaster;
    ros::NodeHandle nh;

    ros::Subscriber sub_left = nh.subscribe<std_msgs::Int16>("lwheel", 10, left_callback);
    ros::Subscriber sub_right = nh.subscribe<std_msgs::Int16>("rwheel", 10, right_callback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(20);

    while(nh.ok()){
        ros::spinOnce();    
        current_time = ros::Time::now();
        
        double dt = (current_time - last_time).toSec();
        double vr = ((right_odom - old_right_odom)/dt)*TICK_DISTANCE;
        double vl = ((left_odom - old_left_odom)/dt)*TICK_DISTANCE;
        double omega = (vr - vl)/WHEEL_DISTANCE;
        double vx, vy;


        if(fabs(vr - vl) <  __DBL_EPSILON__ ){
            x += vr*cos(th)*dt;
            y += vr*sin(th)*dt;
            th = th;

            vx = vr*cos(th);
            vy = vr*sin(th);
            omega = 0;

        }else{
            double ICC_R = (WHEEL_DISTANCE/2)*((vr + vl)/(vr - vl));
            double ICC_X = x - ICC_R*sin(th);
            double ICC_Y = y + ICC_R*cos(th);
            double old_x = x;
            double old_y = y;

            x = cos(omega*dt)*(x - ICC_X) - sin(omega*dt)*(y - ICC_Y) + ICC_X;
            y = sin(omega*dt)*(x - ICC_X) + cos(omega*dt)*(y - ICC_Y) + ICC_Y;

            th += omega*dt;
            vx = (x - old_x)/dt;
            vy = (y - old_y)/dt;

        }
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);
        
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z - omega;

        odom_pub.publish(odom);

        last_time = current_time;
        old_left_odom = left_odom;
        old_right_odom = right_odom;

        r.sleep();
    }
    return 0;
}