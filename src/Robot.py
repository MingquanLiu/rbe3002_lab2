#!/usr/bin/env python

import rospy
import time
import tf
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry





class Robot:

    def __init__(self):
        """"
        Set up the node here

        """
    	rospy.init_node('RobotControl', anonymous=True)
	self.reset_odam()
	## Sets up the cmd_vel publisher and odem_try subscriber and the subscriber for goal setting up in rviz
	self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
	self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_to_pose)

	## Variables for storing the robot moving state
	self.nav_state = False
	self.goal_msg = None
	self.once = False
	

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """

	transGoal = goal
        (self.goal_angle_roll,self.goal_angle_pitch, self.goal_angle_yaw) = tf.transformations.euler_from_quaternion(
                                                                                [transGoal.pose.orientation.x,
                                                                                 transGoal.pose.orientation.y,
                                                                                 transGoal.pose.orientation.z,
                                                                                 transGoal.pose.orientation.w])
	## This function stores the odemtry information of the navigation goal from rviz 2d goal
	self.goalX = transGoal.pose.position.x
	self.goalY = transGoal.pose.position.y
	rospy.loginfo("x:"+str(self.goalX)+" y:"+ str(self.goalY))
	rospy.loginfo("roll:"+str(self.goal_angle_roll)+" pitch:"+ str(self.goal_angle_pitch)+" yaw:"+ str(self.goal_angle_yaw))

	## Set the robot to be navigation mode
	self.change_nav_state()

	## Normal Drive is driving with no arc
	#self.normal_drive()
	## driveAndRotateToGoal() is driving with arc
	self.driveAndRotateToGoal()

    ## Changes the value of self.nav_state
    def change_nav_state(self):
	if self.nav_state:
		self.nav_state = False
	else:
		print("change to true")
		self.nav_state = True
 
   ## Drive straight makes the robot drive in speed for distance
    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive shraight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """
	## Creates the cmd_vel msg for controling the robot movement
	msg = Twist()
	msg.linear.x = speed
	rospy.loginfo(msg)
	rospy.loginfo("in drive straight")
	cx = self.px
	cy = self.py
	self.pub.publish(msg)
	
	## This while loop checks for the distance moved
	while(math.fabs(math.sqrt(math.pow(self.px-cx,2)+math.pow(self.py-cy, 2)))<distance):
		a = 1

	## If the distance is reached, a stop message will be sent
	msg.linear.x = 0
	self.pub.publish(msg)
	




  
## Rotate makes the robot rotate to a certain angle
    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
      """
	if(angle>0):
		speed = 0.5
	else:
		speed = -0.5

	## Creates the cmd_vel msg for controling the robot movement
	msg = Twist()
	msg.angular.z = speed
	rospy.loginfo(msg)
	rospy.loginfo("in rotate")
	ca = self.anglez
	self.pub.publish(msg)

	## This while loop checks for the angle moved
	while(math.fabs(self.anglez-angle)>0.1):
		a = 1
		print(str(self.anglez)+" "+str(ca))

	## If the distance is reached, a stop message will be sent
	newMsg = Twist()
	newMsg.angular.z = 0
	self.pub.publish(newMsg)
	print("published")

## This function calculates the distance difference and angle difference in real time to control the speed and rotate_speed of the robot. if Robot moves
## to the correct position, it is going to be rotated to goal angle.
    def driveAndRotateToGoal(self):
	originalDistance = math.fabs(math.sqrt(math.pow(self.goalX-self.px,2)+math.pow(self.goalY-self.py, 2)))
	while self.once == False:
		if self.nav_state:
			tolerence = 0.2
			x_diff = self.goalX - self.px
			y_diff = self.goalY - self.py


			distance = math.fabs(math.sqrt(math.pow(self.goalX-self.px,2)+math.pow(self.goalY-self.py, 2)))
			drive_speed = self.translate(distance, 0 ,originalDistance, 0,0.4)
			straight_angle = math.atan2(y_diff, x_diff)
			robot_angle = self.anglez
			## Finds the angle difference between robot angle and the direction angle it should go
			angle_diff = math.atan2(math.sin(straight_angle-robot_angle),math.cos(straight_angle-robot_angle))
			if(angle_diff > 0):
				rotate_speed = self.translate(angle_diff, -math.pi, math.pi, -1.0, 1.0)
			if(angle_diff < 0):
				rotate_speed = self.translate(angle_diff, -math.pi, math.pi, -1.0, 1.0)
			#rotate_speed = angle_diff*0.05
			print("Angle Diff:"+str(angle_diff)+" Rotate speed:"+str(rotate_speed))
			if(math.fabs(angle_diff)<0.1):
				rotate_speed = 0
			if(distance<0.1):
				drive_speed = 0
			if(drive_speed == 0):
				self.change_nav_state()
				self.once = True
				rotate_speed = 0
			self.send_drive_msg(drive_speed,rotate_speed)

	time.sleep(2)
	self.rotate(self.goal_angle_yaw)

## Normal drive first rotate to its driving angle and then drive forwad for a certain distance, at the end it rotate to desired angle.
    def normal_drive(self):
	x_diff = self.goalX - self.px
	y_diff = self.goalY - self.py
	straight_angle = math.atan2(y_diff, x_diff)
	robot_angle = self.anglez
	angle_diff = math.atan2(math.sin(straight_angle-robot_angle),math.cos(straight_angle-robot_angle))
	self.rotate(straight_angle)
	time.sleep(3)
	distance = math.sqrt(math.pow(x_diff,2)+math.pow(y_diff, 2))
	self.drive_straight(0.5,distance)
	time.sleep(3)
	self.rotate(self.goal_angle_yaw)

## Sends out a Twist msg with forward speed and rotate speed, if the message was same as before it is not going to be sent 
    def send_drive_msg(self,forward_speed, rotate_speed):
	msg = Twist()
	msg.linear.x = forward_speed
	msg.angular.z = rotate_speed
	if(self.goal_msg is None):
		self.goal_msg = msg
		self.pub.publish(msg)
	else:
		if(msg.linear.x != self.goal_msg.linear.x and msg.angular.z != self.goal_msg.angular.z):
			self.goal_msg = msg
			self.pub.publish(msg)

	
## resets the odom data in the robot object
    def reset_odam(self):
	self.anglex = 0
	self.angley = 0
 	self.anglez = 0
	self.px = 0
	self.py = 0

## Translate is the function mapping from interval [leftMin, leftMax] to [leftMin, leftMax]
    def translate(self,value, leftMin, leftMax, rightMin, rightMax):
	leftSpan = leftMax - leftMin
	rightSpan = rightMax-rightMin
	
	valueScaled = float(value-leftMin)/float(leftSpan)

	return rightMin +(valueScaled * rightSpan)
	
  


## Odom_callback actually record the odom message whenever it is getting sent
    def odom_callback(self, msg):

        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
	self.px = msg.pose.pose.position.x
	self.py = msg.pose.pose.position.y
	quat = msg.pose.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	(self.anglex, self.angley, self.anglez) = tf.transformations.euler_from_quaternion(q)



if __name__ == '__main__':
	## Creates the robot object
	robot = Robot()
	time.sleep(4)
	#robot.rotate(1)
	print("in Main")
	#robot.drive_straight(10, 0.01)
	"""time.sleep(5)
	robot.drive_straight(0, 0)"""
	rospy.spin()

			
 
