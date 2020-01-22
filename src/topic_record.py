#! /usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import Imu

class ImuToFile:
    def __init__(self, topic_name, filename):
        self.topic = topic_name
        self.filename = filename
        self.topic_sub = rospy.Subscriber(self.topic, Imu, self.topic_callback)
        self.topic_data = np.array([])
        self.msg_count = 0
    
    def topic_callback(self, msg):
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        self.topic_data = np.append(self.topic_data, [gyro_x, gyro_y, gyro_z])
        self.msg_count += 1
        print "{} data recieved".format(self.msg_count)
        if self.msg_count > 2000:
            self.save_file()
            rospy.loginfo('data saved')
            self.msg_count = 0
            rospy.signal_shutdown('saved one file')
    
    def save_file(self):
        np.savetxt(self.filename, np.reshape(self.topic_data, (-1,3)))

if __name__ == "__main__":
    rospy.init_node('imu2file')
    topic = '/imu/data'
    f = 'imu_data_filtered.csv'
    n = ImuToFile(topic, f)
    rospy.spin()