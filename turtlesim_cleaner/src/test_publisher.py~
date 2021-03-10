from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy

def change_speed_msg(speed_msg_obj, cmd=' ', std_speed = 5):
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

def initialise_twist(speed_msg_obj):
	speed_msg_obj.linear.y = 0
	speed_msg_obj.linear.z = 0
	speed_msg_obj.linear.x = 0
	speed_msg_obj.angular.x = 0
	speed_msg_obj.angular.y = 0
	speed_msg_obj.angular.z = 0
	return speed_msg_obj

def check_correct_input(cmd):
	return cmd == 'w' or cmd == 's' or cmd == 'a' or cmd== 'd'

def main():
	rospy.init_node('test_subscriber')	
	
	cmd_publisher_0 = rospy.Publisher('elisa3_robot_0/mobile_base/cmd_vel',Twist)
	cmd_publisher_1 = rospy.Publisher('elisa3_robot_1/mobile_base/cmd_vel',Twist)

	while True:
		try:
			cmd_0 = str(raw_input('Robot 0 Movement command (up=w,down=s,left=a, or right=d): '))
			cmd_1 = str(raw_input('Robot 1 / Movement command (up=w,down=s,left=a, or right=d): '))
			speed_msg_0 = initialise_twist(Twist())
			speed_msg_1 = initialise_twist(Twist())

			if cmd_1 == 'exit' and cmd_0 =='exit':
				cmd_publisher_0.publish(speed_msg_0)
				cmd_publisher_1.publish(speed_msg_1)
				return
			if check_correct_input(cmd_0) and not check_correct_input(cmd_1):
				print('robot 1 fout')
				cmd_publisher_1.publish(speed_msg_1)
				speed_msg_0 = change_speed_msg(speed_msg_0, cmd_0)
				cmd_publisher_0.publish(speed_msg_0)
				continue
			if not check_correct_input(cmd_0) and check_correct_input(cmd_1):
				print('robot 0 fout')
				cmd_publisher_0.publish(speed_msg_0)
				speed_msg_1 = change_speed_msg(speed_msg_1, cmd_1)
				cmd_publisher_1.publish(speed_msg_1)
				continue
			if not check_correct_input(cmd_0) and not check_correct_input(cmd_1):
				cmd_publisher_0.publish(speed_msg_0)
				cmd_publisher_1.publish(speed_msg_1)
				print('beide fout')
				continue
			else:
				print('beide goed')
				speed_msg_1 = change_speed_msg(speed_msg_1, cmd_1)
				speed_msg_0 = change_speed_msg(speed_msg_0, cmd_0)
				cmd_publisher_0.publish(speed_msg_0)
				cmd_publisher_1.publish(speed_msg_1)
				
		except:
			print('failure')


if __name__ == '__main__' :
	main()


