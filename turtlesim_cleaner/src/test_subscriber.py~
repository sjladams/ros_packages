from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import rospy

def callback(pose):
	#rospy.loginfo(rospy.get_caller_id() + 'I hear %s', pose.pose)
	print(str(pose.pose.position.x))
	return 

def main():
	try:
		rospy.init_node('test_subscriber')
		cmd_listener = rospy.Subscriber("/rigid_body_1/pose", PoseStamped, callback)
		rospy.spin()
	except:
		print('undefined failure')


if __name__ == '__main__' :
	main()


