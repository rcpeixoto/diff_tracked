#!/usr/bin/python

import smbus
import rospy

from sensor_msgs.msg import Range 
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Vector3
bus = smbus.SMBus(1)


def callback(data):
    cmd = data.data
    count = 0
    while count < len(cmd):
        bus.write_byte(0x12, cmd[count])
        count = count + 1

def talker():
    rospy.init_node("i2c_comns", anonymous=True)

    wheel_speed = rospy.Subscriber("cmd_msg", String, callback)

    imu = rospy.Publisher("imu_raw_data", Imu, queue_size=10)
    lwheel = rospy.Publisher("lwheel", Int16, queue_size=10)
    rwheel = rospy.Publisher("rwheel", Int16, queue_size=10)
    sonar = rospy.Publisher("sonar", Range, queue_size=10)
    
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        sonar_data = ((bus.read_byte(0x12) << 8) & 0xff00) +  bus.read_byte(0x12)

        AcX = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)
        AcY = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)
        AcZ = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)
        GyX = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)
        GyY = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)
        GyZ = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)

        leftCount = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)
        rightCount = ((bus.read_byte(0x12) << 8) & 0xff00) | bus.read_byte(0x12)
        
        msg = Int16()
        msg.data = leftCount
        lwheel.publish(msg)

        msg = Int16()
        msg.data = rightCount
        rwheel.publish(msg)

        msg = Range()
        msg.header.frame_id = "sonar_link"
        msg.header.stamp = rospy.Time.now()
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.523559
        msg.min_range = 0.02
        msg.max_range = 4
        msg.range = sonar_data/100

        sonar.publish(msg)

        accel = Vector3()
        accel.x = AcY
        accel.y = AcY
        accel.z = AcZ

        gyro = Vector3()
        gyro.x = GyX
        gyro.y = GyY
        gyro.z = GyZ

        imu_data = Imu()
        imu_data.linear_acceleration = accel
        imu_data.angular_velocity = gyro
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = "mpu_link"
        
        imu.publish(imu_data)

        rospy.spin()
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
