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
	
	#cmd_publisher_0 = rospy.Publisher('elisa3_robot_0/mobile_base/cmd_vel',Twist)
	cmd_publisher_1 = rospy.Publisher('elisa3_robot_1/mobile_base/cmd_vel',Twist)
	while True:
		try:
			#cmd_0 = str(raw_input('Robot 0 Movement command (up=w,down=s,left=a, or right=d): '))
			cmd_1 = str(raw_input('Robot 1 / Movement command (up=w,down=s,left=a, or right=d): '))
			#speed_msg_0 = Twist()
			speed_msg_1 = Twist()

			if cmd_1 == 'exit':
				#cmd_publisher_0.publish(speed_msg_0)
				cmd_publisher_1.publish(speed_msg_1)
				return
			if check_correct_input(cmd_1):
				continue
			else:
				speed_msg_1 = change_speed_msg(speed_msg_1, cmd_1)
				#speed_msg_0 = change_speed_msg(speed_msg_0, cmd_0)

			#cmd_publisher_0.publish(speed_msg_0)
			print('tot hier')
			cmd_publisher_1.publish(speed_msg_1)
				
		except:
			print('failure')


if __name__ == '__main__' :
	main()


