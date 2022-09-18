#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

extern "C"{
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



#define PIC_ADDR 0x12
#define IMU_ADDR 0x68

__u8 *cmd;


void callback(const std_msgs::String::ConstPtr& msg){
    cmd = msg->data;
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

    geometry_msgs::Vector3 accel;
    geometry_msgs::Vector3 gyro;
    sensor_msgs::Imu imu;
    
    //Opens i2c bus
    int file;
    char filename[20];
    snprintf(filename, 20, "/dev/i2c-%d", 1);
    if((file = open(filename, O_RDWR)) < 0) exit(1);

    //Sets IMU as slave
    if(ioctl(file, I2C_SLAVE, IMU_ADDR) < 0) exit(1);

    i2c_smbus_write_byte_data(file, 0x6b, 0x00);
    i2c_smbus_write_byte_data(file, 0x1b, 0x00);
    i2c_smbus_write_byte_data(file, 0x1c, 0x00);
    i2c_smbus_write_byte_data(file, 0x37, 0x90);
    i2c_smbus_write_byte_data(file, 0x38, 0x01);



    while(nh.ok()){
	if(ioctl(file, I2C_SLAVE, IMU_ADDR) < 0) exit(1);
        __u8 data[14];
        __u8 reg = 0x3b;
	__u8 length = 14;
        i2c_smbus_read_i2c_block_data(file, reg, length, data);

        accel.x = ((short int) (data[0] << 8) | data[1])/16384.0;
        accel.y = ((short int) (data[2] << 8) | data[3])/16384.0;
        accel.z = ((short int) (data[4] << 8) | data[5])/16384.0;

        gyro.x = ((short int) (data[8] << 8) | data[9])/131.0;
        gyro.y = ((short int) (data[10] << 8) | data[11])/131.0;
        gyro.z = ((short int) (data[12] << 8) | data[13])/131.0;

        imu.angular_velocity = gyro;
        imu.linear_acceleration = accel;
	imu.header.frame_id = "mpu_link";
	imu.header.stamp = ros::Time::now();        

        if(ioctl(file, I2C_SLAVE, PIC_ADDR) < 0) exit(1);
        
        int sonar_data = (int) (i2c_smbus_read_byte(file) << 8) | i2c_smbus_read_byte(file);
        std_msgs::Int16 left_count;
        left_count.data = (int) (i2c_smbus_read_byte(file) << 8) | i2c_smbus_read_byte(file);
        std_msgs::Int16 right_count;
        right_count.data = (int) (i2c_smbus_read_byte(file) << 8) | i2c_smbus_read_byte(file);    
        //Publishes data from pic
        sensor_msgs::Range msg;
        msg.header.frame_id = "sonar_link";
        msg.header.stamp = ros::Time::now();
        msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        msg.field_of_view = 0.523559;
        msg.min_range = 0.02;
        msg.max_range = 4;
        msg.range = sonar_data/100.0;

        //Writes velocity commands through i2c
        __u8 *ptr = cmd;
        while(ptr != NULL)
                i2c_smbus_write_byte(file, ptr++);
        

        sonar.publish(msg);
        left_wheel.publish(left_count);
        right_wheel.publish(right_count);
        imu_data.publish(imu);
        ros::spinOnce();
        r.sleep();
    }
    close(file);
    return 0;
}

