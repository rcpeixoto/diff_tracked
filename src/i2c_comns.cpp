#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



#define PIC_ADDR 0x12
#define IMU_ADDR 0x68


void callback(const std_msgs::String::ConstPtr& msg){
    
}


int main(int argc, char** argv){
    ros::init(argc, argv, "i2c_comns");
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel = nh.subscribe<std_msgs::String>("cmd_msg", 10, callback);

    ros::Publisher sonar = nh.advertise<sensor_msgs::Range>("sonar", 10);
    ros::Publisher left_wheel = nh.advertise<std_msgs::Int16>("lwheel", 10);
    ros::Publisher right_wheel = nh.advertise<std_msgs::Int16>("rwheel", 10);
    ros::Publisher imu_data = nh.advertise<sensor_msgs::Imu>("imu", 10);

    ros::Rate r(4);

    int file;
    char filename[20];
    snprintf(filename, 20, "/dev/i2c-%d", 1);

    if((file = open(filename, O_RDWR)) < 0) {
        exit(1);
    }


    while(nh.ok()){
        //Reads data from Pic
        if(ioctl(file, I2C_SLAVE, PIC_ADDR) < 0){
            exit(1);
        }   
        char buf[6];
        read(file, buf, 6);
        int sonar_data = (int) (buf[0] << 8) | buf[1];
        std_msgs::Int16 left_count;
        left_count.data = (int) (buf[2] << 8) | buf[3];
        std_msgs::Int16 right_count;
        right_count.data = (int) (buf[4] << 8) | buf[5];    
        //Publishes data from pic
        sensor_msgs::Range msg;
        msg.header.frame_id = "sonar_link";
        msg.header.stamp = ros::Time::now();
        msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        msg.field_of_view = 0.523559;
        msg.min_range = 0.02;
        msg.max_range = 4;
        msg.range = sonar_data/100;

        sonar.publish(msg);
        left_wheel.publish(left_count);
        right_wheel.publish(right_count);

        //Reads from IMU


        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
