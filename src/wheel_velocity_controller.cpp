#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#define WHEEL_DISTANCE 0.23

double vel = 0;
double omega = 0;
/*
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    double vel = msg->linear.x;
    double omega = msg->angular.z;
    double rightWheel = (vel + omega*(WHEEL_DISTANCE/2));
    double leftWheel  = (vel - omega*(WHEEL_DISTANCE/2));

    int left_wheel_cmd = -2699910*pow(leftWheel, 6) + 3045080*pow(leftWheel, 5) + 115204*pow(leftWheel, 4) - 180807*pow(leftWheel, 3) - 1179.32*pow(leftWheel, 2) + 4485.28*leftWheel + 1500;
    int right_wheel_cmd = -2699910*pow(rightWheel, 6) + 3045080*pow(rightWheel, 5) + 115204*pow(rightWheel, 4) - 180807*pow(rightWheel, 3) - 1179.32*pow(rightWheel, 2) + 4485.28*rightWheel + 1500;

    int rightFoward = (right_wheel_cmd > 1500? 1 : 0);
    int leftFoward = (left_wheel_cmd > 1500? 1 : 0);

    std::string cmd = "#12P" + std::to_string(left_wheel_cmd) +"T1000#13P" + 
        std::to_string(right_wheel_cmd) + "T1000\r" + ";"+ std::to_string(rightFoward) + ";" + std::to_string(leftFoward);

    std_msgs::String cmd_send;
    cmd_send.data = cmd;

    pub_cmd_vel.publish(cmd_send);

}
*/
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    vel = msg->linear.x;
    omega = msg->angular.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "wheel_velocity_controller");
    ros::NodeHandle n;
    ros::Publisher pub_cmd_vel = n.advertise<std_msgs::String>("cmd_msg", 100000000);
    ros::Subscriber sub_cmd_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10000000, cmd_vel_callback);
    ros::Rate r(5);

    while(n.ok()){
        double rightWheel = (vel + omega*(WHEEL_DISTANCE/2));
        double leftWheel  = (vel - omega*(WHEEL_DISTANCE/2));

        int left_wheel_cmd = -2699910*pow(leftWheel, 6) + 3045080*pow(leftWheel, 5) + 
            115204*pow(leftWheel, 4) - 180807*pow(leftWheel, 3) - 1179.32*pow(leftWheel, 2) + 4485.28*leftWheel + 1500;
        int right_wheel_cmd = -2699910*pow(rightWheel, 6) + 3045080*pow(rightWheel, 5) +
            115204*pow(rightWheel, 4) - 180807*pow(rightWheel, 3) - 1179.32*pow(rightWheel, 2) + 4485.28*rightWheel + 1500;

        int rightFoward = (right_wheel_cmd > 1500? 1 : 0);
        int leftFoward = (left_wheel_cmd > 1500? 1 : 0);

        std::string cmd = "#12P" + std::to_string(left_wheel_cmd) +"T1000#13P" + 
            std::to_string(right_wheel_cmd) + "T1000\r" + std::to_string(rightFoward) + std::to_string(leftFoward);

        std_msgs::String cmd_send;
        cmd_send.data = cmd;

        pub_cmd_vel.publish(cmd_send);
        ros::spinOnce();
        r.sleep();

    }
    return 0;
}