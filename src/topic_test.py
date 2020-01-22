import rospy
from std_msgs.msg import String

rospy.init_node('topic_pub')
pub = rospy.Publisher('Pub_test', String, queue_size=1)
rate = rospy.Rate(10)
msg = 'abc'

while not rospy.is_shutdown():
	pub.publish(msg)
	rate.sleep()
