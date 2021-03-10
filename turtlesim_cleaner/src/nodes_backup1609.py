from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import rospy
import numpy as np
from functions import *
import time

wheel_distance = 0.041

class Node:
    def __init__(self, rigid_body_id, controll_id):
        self.rigid_body_id = rigid_body_id
        self.controll_id = controll_id

        self.pose = np.array([np.NaN, np.NaN])
        self.orien = np.NaN

        rospy.init_node('python_node_' + str(controll_id))
        self.listener_pose = rospy.Subscriber("/rigid_body_"+str(self.rigid_body_id)+
                            "/pose", PoseStamped, self.listener_pose_callback)
        self.publisher_motor_vlt = rospy.Publisher('elisa3_robot/mobile_base/cmd_vel',Float64MultiArray)

        self.motor_vlt_msg = Float64MultiArray()
        self.green_led_msg = Float64MultiArray()

        self.std_speed = 0.045   #[m/s]
        self.std_vlt = 10

        self.error_angle = 0
        self.rel_move = 0
        self.corr_abs_move = 0


    def listener_pose_callback(self,pose):
        self.pose = np.array([float(pose.pose.position.x),float(pose.pose.position.y)])
        self.orien = float(pose.pose.orientation.z)

    def publish_motor_vlt(self, vlt = np.array([0,0])): #leftVoltage, rightVoltage
        self.motor_vlt_msg.data = np.append(np.array([self.controll_id]), vlt)
        self.publisher_motor_vlt.publish(self.motor_vlt_msg)


    def move(self, rel_pol_move=np.array([0,0])): #[rho, phi]
        # turn right direction
        self.publish_motor_vlt(np.array([-self.std_vlt, self.std_vlt]))
        time.sleep((rel_pol_move[1]*wheel_distance/2)/self.std_speed)
        self.publish_motor_vlt()

        # move straight
        self.publish_motor_vlt(np.array([self.std_vlt, self.std_vlt]))
        time.sleep(rel_pol_move[0]/self.std_speed)
        self.publish_motor_vlt()


    def callibrate(self):
        call_time = 2.5

        # relative callibration
        start_time = time.time()
        start_pose = self.pose
        self.publish_motor_vlt(np.array([self.std_vlt, self.std_vlt]))
        time.sleep(call_time)
        end_time = time.time()
        self.publish_motor_vlt()
        end_pose = self.pose

        self.publish_motor_vlt(np.array([-self.std_vlt, -self.std_vlt]))
        time.sleep(call_time)
        self.publish_motor_vlt()

        norm_meas_move = normalize(end_pose-start_pose)
        self.error_angle = angle_vectors(np.array([1,0]), norm_meas_move)
        self.rel_move_diff = wheel_distance * self.error_angle

        # abs callibration
        start_time = time.time()
        start_pose = self.pose
        self.publish_motor_vlt(np.array([self.std_vlt, self.std_vlt]))
        time.sleep(call_time)
        end_time = time.time()
        self.publish_motor_vlt()
        end_pose = self.pose
        meas_move = end_pose-start_pose
        meas_speed = np.linalg.norm(meas_move)/call_time
        self.corr_abs_move = 1 - meas_speed / self.std_speed

        # test abs callibration
        start_pose = self.pose
        self.publish_motor_vlt(np.array([self.std_vlt*(1+self.corr_abs_move),
                                         self.std_vlt*(1+self.corr_abs_move)]))
        time.sleep(call_time)
        end_time = time.time()
        self.publish_motor_vlt()
        end_pose = self.pose
        meas_move = end_pose-start_pose
        meas_speed = np.linalg.norm(meas_move)/call_time










class Nodes:
    def __init__(self, mapper_dict = {0: 1}):
        self.nodes = {controller_id: Node(mapper_dict[controller_id], controller_id) for
                      controller_id in mapper_dict}






