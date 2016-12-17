#!/usr/bin/env python

# Code Done by Emanuele Sansebastiano, November 2016
# International System Unit (ISU) is used

import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from std_msgs.msg import Empty

rospy.init_node('turtlebot_controller')

# initialization
global x_od, y_od, th_od
x_od = 1; y_od = 1; th_od = 1;

# initialization of motion
exit = 0

# pid = 0 --> the flying robot is stil on the mobile platform
# pid = 1 --> the flying robot is flying
# pid = 2 --> the flying robot landed (go home)
global pid, start_val
pid = 0; start_val = 0


msg = """
This code make the turtlebot move on a fixed path
It will collaborate with a parrot 2.0

In order to start the protocol type in another terminal

rosrun move_sans_dipen key_commander.py

CTRL-C to quit
"""

# speed definition
speed = 0.2; turn = 1
global speed_max, turn_max
speed_max = 0.4; turn_max = 1.5

# initialization
control_speed = 0; control_turn = 0
# end speed definition

# time step execution
time_step = 0.4

# Function to subscribe
def callback(msg_read):
	global pid 
	pid = msg_read.data

def callback_start(msg_read):
	global start_val 
	start_val = msg_read.data

def odom_listener(msg_pose):
	global x_od, y_od, th_od
 
	pose_rob = msg_pose.pose.pose
	x_od = pose_rob.position.x
	y_od = pose_rob.position.y
	th_od = pose_rob.orientation.z

# end function to subscribe callbacks definition

def go_home1(speed, turn, time_step):
	# open loop control (odometry based)
	global x_od, y_od, th_od
	global speed_max, turn_max
	exit = 0

	# gains to be tuned
	k_speed1 = 0.08; k_turn1 = 0.5

#	rospy.Subscriber('/vestito/odom',Odometry,odom_listener)
	rospy.Subscriber('odom',Odometry,odom_listener)

	# spatial coordinates definition
	x_init = 0; x_init = 0; th_init = 0
	x_des = x_init; y_des = x_init 
	x_cur = x_od; y_cur = y_od; th_cur = th_od 
	x_err = x_cur - x_des
	y_err = y_cur - y_des
	rho_err = math.sqrt(math.pow(x_err,2) + math.pow(y_err,2))

#	# the orientation is defined as percentage of pi
#	th_des = (math.atan2(- y_err, x_err) + math.pi/2)/math.pi
	# the orientation is defined as percentage of pi
	th_des = (math.atan2( y_err, x_err))/math.pi
	th_des = th_des + (math.pi)/math.pi

	if th_des > (math.pi)/math.pi:
		th_des = th_des - (2*math.pi)/math.pi

	th_err = th_cur - th_des

	# velocity definition
	# go the the home point
	if rho_err > 0.1:
		speed_home = k_speed1 * rho_err
		if speed_home < 0.1 and math.fabs(th_err) < 0.4:
			speed_home = 0.1
		
		#to adjust th_err
		const = 0.0
		if y_od < 0:
			const = -const
		th_err = th_err +const
		turn_home = k_turn1 * th_err
	# turn into the initial direction
	else:
		th_err = th_cur - th_init
		speed_home = k_speed1 * rho_err *0
		turn_home = math.fabs(k_turn1 * th_err)

		# conclusion of the task
		if math.fabs(th_err) < 0.02:
			exit = 1
			print 'MISSION COMPLETED!'
	
	# limitation to speed
	if speed_home > speed_max:
		speed_home = speed_max
	if turn_home > turn_max:
		turn_home = turn_max

#	print x_od, y_od, th_od
#	print rho_err, th_err
#	print speed_home, turn_home		

	pid_vel = (speed_home/speed, turn_home/turn, exit)
	return pid_vel
# end function go_home1

def go_home2(speed, turn, time_step):
	# open loop control (odometry based)
	global x_od, y_od, th_od
	global speed_max, turn_max
	exit = 0

	# gains to be tuned
	k_speed2 = 1.2; k_turn2 = 0.3

#	rospy.Subscriber('/vestito/odom',Odometry,odom_listener)
	rospy.Subscriber('odom',Odometry,odom_listener)

	# spatial coordinates definition
	x_init = 0; x_init = 0; th_init = 0
	x_des = x_init; y_des = x_init 
	x_cur = x_od; y_cur = y_od; th_cur = th_od 
	x_err = x_cur - x_des
	y_err = y_cur - y_des
	rho_err = math.sqrt(math.pow(x_err,2) + math.pow(y_err,2))

#	# the orientation is defined as percentage of pi
#	th_des = (math.atan2(- y_err, x_err) + math.pi/2)/math.pi
	# the orientation is defined as percentage of pi
	th_des = (math.atan2( y_err, x_err))/math.pi
	th_des = th_des + (math.pi)/math.pi

	if th_des > (math.pi)/math.pi:
		th_des = th_des - (2*math.pi)/math.pi

	th_err = th_cur - th_des

	# velocity definition
	# go the the home point
	if rho_err > 0.09:

		#to adjust th_err
		const = 0.0		
		if y_od < 0:
			const = -const
		th_err = th_err +const

		if math.fabs(th_err) > 0.08:
	
			speed_home = 0
			turn_home = k_turn2 * th_err
		else:
			speed_home = k_speed2 * rho_err
			if speed_home < 0.1 and rho_err > 0.15:
				speed_home = 0.1
			turn_home = 0
		
	# turn into the initial direction
	else:
		th_err = th_cur - th_init
		speed_home = 0
		turn_home = math.fabs(k_turn2 * th_err)

		# conclusion of the task
		if math.fabs(th_err) < 0.05:
			exit = 1
			print 'MISSION COMPLETED!'
	
	# limitation to speed
	if speed_home > speed_max:
		speed_home = speed_max
	if turn_home > turn_max:
		turn_home = turn_max

#	print x_od, y_od, th_od
#	print rho_err, th_err
#	print speed_home, turn_home		

	pid_vel = (speed_home/speed, turn_home/turn, exit)
	return pid_vel
# end function go_home2


def go_one(speed, turn, time_step, two_c, three_c, count_loc):
	
	speed_one = 0.2; turn_one = 0

	if count_loc > two_c/time_step:
		speed_one = 0; turn_one = 1

	if count_loc > three_c/time_step:
#		speed_one = 0.27; turn_one = -0.35
		speed_one = 0.15; turn_one = -0.25


	pid_vel = (speed_one/speed, turn_one/turn)
	return pid_vel
# end function go_one 



count_stop = 0
count_loc = -1
count_limit_max = 100000

#
# ---------------------------------------------------------------------------------------
# main

print msg

#pub = rospy.Publisher('/vestito/cmd_vel_mux/input/teleop', Twist, queue_size=5)
#pub_odom = rospy.Publisher('/vestito/mobile_base/commands/reset_odometry',Empty, queue_size=5)
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
pub_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty, queue_size=5)

pub_init_parr = rospy.Publisher('parrot_status',Int8, queue_size=8)
pub_init_turt = rospy.Publisher("turtlebot_start", Int8, queue_size=8)

# to be sure everything is fine
rospy.sleep(1)

twist = Twist()
twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
pub.publish(twist)

# to initialize topic values
pub_odom.publish()
pub_init_parr.publish(pid)
pub_init_turt.publish(start_val)

# while to lock the starting process
temp = 1
while start_val == 0 and not rospy.is_shutdown():

	rospy.sleep(time_step/4)
	rospy.Subscriber('turtlebot_start', Int8, callback_start)

	while temp == 1:
		print 'Turtlebot is waiting...'
		temp = 0	
# end of while

if not rospy.is_shutdown():
    print 'Turtlebot process fired!'

# to move
while exit == 0 and not rospy.is_shutdown():

	rospy.Subscriber('parrot_status', Int8, callback)
	
	# go home
	if pid > 1:

		pid_vel = go_home1(speed, turn, time_step)
#		pid_vel = go_home2(speed, turn, time_step)

		pid_speed = pid_vel[0]
		pid_turn = pid_vel[1]
		exit = pid_vel[2]

		control_speed = speed * pid_speed
		control_turn = turn * pid_turn

	# normal motion 
	else:
		# just one time
		while count_loc == -1:
			count_loc = 0
			two_c = 5; three_c = 6

		pid_vel = go_one(speed, turn, time_step, two_c, three_c, count_loc)
		pid_speed = pid_vel[0]
		pid_turn = pid_vel[1]
		
		count_loc = count_loc +1
		if count_loc == count_limit_max:
			count_loc = count_limit_max
		
		control_speed = speed * pid_speed
		control_turn = turn * pid_turn

		if pid == 1:
			control_speed = control_speed *0
			control_turn = control_turn *0

			#uncomment the following line to reset the odometry
			#when the drone takes off or lands
			#pub_odom.publish()

	# execution of the previous layer related to robot velocity
	twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
	pub.publish(twist)

	rospy.sleep(time_step)
# end of while

# to stop slowly
count_stop = 0
dividing_factor = 1.1
while count_stop < 7:

	control_speed = control_speed/dividing_factor
	control_turn = control_turn/dividing_factor

	twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
	pub.publish(twist)

	rospy.sleep(time_step)

	count_stop = count_stop +1
# end of while

# definite stop
twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
pub.publish(twist)

print 'Drone disarmed'

# end of main
