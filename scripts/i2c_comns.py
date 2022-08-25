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
    bus.write_i2c_block_data(0x08, 0x00, data.data)

def talker():
    rospy.init_node("i2c_comns", anonymous=True)
    wheel_speed = rospy.Subscriber("cmd_msg", String, callback)
    imu = rospy.Publisher("imu_raw_data", Imu, queue_size=10)
    lwheel = rospy.Publisher("lwheel", Int16, queue_size=10)
    rwheel = rospy.Publisher("rwheel", Int16, queue_size=10)
    sonar = rospy.Publisher("sonar", Range, queue_size=10)
    
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        

        rospy.spin()
        r.sleep()





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
