#!/usr/bin/python

import smbus
import rospy

from sensor_msgs.msg import Range 
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3
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

    lwheel = rospy.Publisher("lwheel", Int16, queue_size=10)
    rwheel = rospy.Publisher("rwheel", Int16, queue_size=10)
    sonar = rospy.Publisher("sonar", Range, queue_size=10)
    
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        sonar_data = (bus.read_byte(0x12) << 8) +  bus.read_byte(0x12)
        leftCount = (bus.read_byte(0x12) << 8) + bus.read_byte(0x12)
        rightCount = (bus.read_byte(0x12) << 8) + bus.read_byte(0x12)
        
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

        rospy.spin()
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
