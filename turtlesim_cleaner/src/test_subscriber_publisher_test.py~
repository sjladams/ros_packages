from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy

def change_speed_msg(speed_msg_obj, cmd=' ', std_speed = 10):
	speed_msg_obj.linear.y = 0
	speed_msg_obj.linear.z = 0
	speed_msg_obj.linear.x = 0
	speed_msg_obj.angular.x = 0
	speed_msg_obj.angular.y = 0
	speed_msg_obj.angular.z = 0

	if cmd == 'w':
		speed_msg_obj.linear.x = std_speed
	if cmd == 's':
		speed_msg_obj.linear.x = -std_speed
	if cmd == 'a':
		speed_msg_obj.linear.y = std_speed
	if cmd == 'd':
		speed_msg_obj.linear.y = -std_speed 
	return speed_msg_obj

def check_correct_input(cmd):
	return cmd != 'w' and cmd != 's' and cmd != 'a' and cmd!= 'd'

def main():
	rospy.init_node('test_subscriber')	
	
	cmd_publisher = rospy.Publisher('/mobile_base/cmd_vel',Twist)
	while True:
		try:
			cmd = str(raw_input('Robot Movement command (up=w,down=s,left=a, or right=d): '))
			speed_msg = Twist()

			if cmd == 'exit':
				cmd_publisher.publish(speed_msg)
				return
			if check_correct_input(cmd):
				continue
			else:
				speed_msg = change_speed_msg(speed_msg, cmd)

			cmd_publisher.publish(speed_msg)
				
		except:
			print('failure')


if __name__ == '__main__' :
	main()


