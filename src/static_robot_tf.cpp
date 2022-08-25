
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "diff_transform_broadcaster");
    ros::NodeHandle nh;

    ros::Rate r(20);

    tf::TransformBroadcaster broadcaster;

    while(nh.ok()){
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0.08)), ros::Time::now(), "base_link", "mpu_link"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.06, 0.08, -0.11)), ros::Time::now(), "base_link", "left_wheel"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.06, 0.08, 0.11)), ros::Time::now(), "base_link", "right_wheel"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.05, 0, 0.055)), ros::Time::now(), "base_link", "sonar_link"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.05, 0, 0.065)), ros::Time::now(), "base_link", "scan_link"));
        r.sleep();
    }
}
