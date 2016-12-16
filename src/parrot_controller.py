#!/usr/bin/env python

# Code Done by Emanuele Sansebastiano & Dipendra Subedi, December 2016
# International System Unit (ISU) is used

import rospy
import sys
import math

from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ardrone_autonomy.msg import Navdata

rospy.init_node('parrot_controller', anonymous=True)

global start_val, vel_sum, z_od, tag_detection
z_od_init = 1000
start_val = 0; vel_sum = 1000; z_od = z_od_init; tag_detection = 0

def callback(vel):
	global vel_sum
	vel_sum = vel.angular.x + vel.angular.y + vel.angular.z + vel.linear.x + vel.linear.y + vel.linear.z

def odom_listener(msg_pose):
	global z_od
	pose_rob = msg_pose.pose.pose
	z_od = pose_rob.position.z

#callback to land smoothly
def callback_land(data):
	twist = Twist()
	if (not data.tags_distance):
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0;
		twist.angular.x = 0; 
		twist.angular.y = 0; 
		twist.angular.z = 0;
		pub_vel_twist.publish(twist)		
	else:
# Uncomment to make it land smoothly
#		twist.linear.x = 0
#		twist.linear.y = 0
#		twist.linear.z = -0.1;
#		twist.angular.x = 0; 
#		twist.angular.y = 0; 
#		twist.angular.z = 0;	
#		pub_vel_twist.publish(twist)
#		if(data.tags_distance[0] < 60):
#			pub_land = rospy.Publisher("ardrone/land", Empty, queue_size=10)
#			pub_land.publish(Empty())
		pub_land = rospy.Publisher("ardrone/land", Empty, queue_size=10)
		rospy.sleep(2)
		pub_land.publish(Empty())

#square protocol
def square_protocol(velx,vely):
	# velx and vely must be '-1', '0' or '1'
 
	twist = Twist()
	velocity = 0.1
	twist.linear.x = velocity*velx;
	twist.linear.y = velocity*vely;
	twist.linear.z = 0;
	twist.angular.x = 0; 
	twist.angular.y = 0; 
	twist.angular.z = 0;
	pub_vel_twist.publish(twist)

	rospy.sleep(1)

	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0; 
	twist.angular.y = 0; 
	twist.angular.z = 0;
	pub_vel_twist.publish(twist)




def callback_tag(data):
	global tag_detection
	if (not data.tags_distance):
		tag_detection = 0
	else:
		tag_detection = 1

def callback_start(msg_read):
	global start_val 
	start_val = msg_read.data

time_step = 0.1
#
#-------------------------------------------------------------------------------------------
#main

#pub_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty, queue_size=5)
pub_parr = rospy.Publisher('parrot_status',Int8, queue_size=10)
pub_init_turt = rospy.Publisher("turtlebot_start", Int8, queue_size=10)
pub_vel_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=10 )
pub_takeoff = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10)
pub_land = rospy.Publisher("ardrone/land", Empty, queue_size=10)

# to be sure everything is fine
rospy.sleep(1)

# to initialize topic values
pid = 0
#pub_odom.publish()
pub_parr.publish(pid)
pub_init_turt.publish(start_val)


# while to lock the starting process
temp = 1
while start_val == 0 and not rospy.is_shutdown():

	rospy.Subscriber('turtlebot_start', Int8, callback_start)
	rospy.sleep(time_step/4)

	while temp == 1:
		print 'Drone is waiting...'
		temp = 0	
# end of while

print 'Drone process fired!'

sequence_val = 0
counter = 0
time_wait = 15 #15
time_wait2 = 38 #50
counter_limit = time_wait/time_step
temp_pid = 0

while sequence_val < 3 and not rospy.is_shutdown():
	
	# protocol chooser
	if counter == counter_limit:
		sequence_val = 1
		if temp_pid == 1:
			sequence_val = 2
			temp_pid = 1
		temp_pid = 1
	
	# taking off protocol
	if sequence_val == 1:
		pid = 1
		
		# while the height of the drone from the ground is not fixed stay in the loop
		height_pid = 1000
		tag_pid = 0; tag_detection = 0; tag_find_counter = 0; temp_times = -1
		while (height_pid > 0.005 or tag_pid == 0) and not rospy.is_shutdown():
			# while the turtlebot is moving (|velocity| > 0) do not take off
			velocity_pid = height_pid/1000
			while velocity_pid > 0.01 and not rospy.is_shutdown():
				rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, callback)
				rospy.sleep(time_step)
				velocity_pid = vel_sum			
				pub_parr.publish(pid)

			# turtlebot is finally stopped, so take off			
			pub_parr.publish(pid)
			pub_takeoff.publish(Empty())

			#one time wait
			counter = 0
			while counter < 1:
				rospy.sleep(4)
				counter = counter +1

			# parrot height odometry listener
			rospy.Subscriber('ardrone/odometry',Odometry,odom_listener)
			rospy.sleep(time_step/2)
			height_temp1 = z_od

			# to be sure z_od is now the initialized one
			if z_od == z_od_init:
				height_temp1 = z_od/2

			rospy.Subscriber('ardrone/odometry',Odometry,odom_listener)
			rospy.sleep(time_step/2)
			height_temp2 = z_od
			height_pid = math.fabs(height_temp1 - height_temp2)

			#tag detection
			# if to exit the process after some attempts
			if tag_find_counter > 10: #7:
				pub_land = rospy.Publisher("ardrone/land", Empty, queue_size=10)
				print 'the drone cannot meet the tag - system down'
				print 'turtlebot is sent home and the drone lands'
				rospy.sleep(1)
				pid = 2
				pub_parr.publish(pid)
				pub_land.publish(Empty())
				rospy.sleep(1)
				pub_parr.publish(pid)
				pub_land.publish(Empty())
				sys.exit()

			# if statement to look for the tag 
			if tag_find_counter > 3:
				print 'square protocol to find the tag is active'
				
				#this is applicated just one time
				if temp_times == -1:
					pid_vel_x = 1; pid_vel_y = 1
					square_protocol(pid_vel_x, pid_vel_y)
					
				#this regolates the direction of the drone looking for the tag
				#in particular it is spanning the edges of a square
				else: 	
					temp_chooser = tag_find_counter % 4
					if temp_chooser == 0:
						pid_vel_x = -1; pid_vel_y = 0
					elif temp_chooser == 1:
						pid_vel_x = 0; pid_vel_y = -1
					elif temp_chooser == 2:
						pid_vel_x = 1; pid_vel_y = 0
					else:
						pid_vel_x = 0; pid_vel_y = 1
					
					square_protocol(pid_vel_x, pid_vel_y)
					print temp_chooser, 'vel x', pid_vel_x, 'vel y', pid_vel_y
				
				#tag detection part
				rospy.Subscriber("/ardrone/navdata", Navdata, callback_tag)
				rospy.sleep(time_step/2)			
				tag_pid = tag_detection

				temp_times = temp_times +1

				#everytime temp_times go became 2 it while refreshed to 0 
				#and the tag_find_counter increase by 1
				if temp_times > 1:
					temp_times = 0
					tag_find_counter = tag_find_counter +1
			
			# preliminary attempt to find the tag	
			else:
				print 'taking off'
			
				#tag detection part
				rospy.Subscriber("/ardrone/navdata", Navdata, callback_tag)
				rospy.sleep(time_step/2)			
				tag_pid = tag_detection

				tag_find_counter = tag_find_counter +1

		tag_find_counter = 0
		counter = 0
		sequence_val = 0
		#change the time to wait in order to land
		time_wait = time_wait2
		counter_limit = time_wait/time_step

		rospy.sleep(time_step)

	# landing protocol and going home 
	elif sequence_val == 2:
		pid = 1
		# while the height of the drone from the ground is more than 0 stay in the loop
		height_pid = 1000
		tag_pid = 0; tag_detection = 0; tag_find_counter = 0
		while height_pid > 1/1000 and not rospy.is_shutdown():
			# while the turtlebot is moving (|velocity| > 0) do not take off
			velocity_pid = height_pid/1000
			while (velocity_pid > 0.01 or tag_pid == 0) and not rospy.is_shutdown():
				rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, callback)
				rospy.sleep(time_step)				
				velocity_pid = vel_sum			
				pub_parr.publish(pid)

				#tag detection
				rospy.Subscriber("/ardrone/navdata", Navdata, callback_tag)
				rospy.sleep(time_step/2)			
				tag_pid = tag_detection

				# if the tag is not detect for more than 't_wait_tag' shoot down all
				t_wait_tag = 4
				if tag_pid == 0:
					tag_find_counter = tag_find_counter +1
				if tag_find_counter > t_wait_tag/time_step:
					pub_land = rospy.Publisher("ardrone/land", Empty, queue_size=10)
					print 'the drone cannot meet the tag - system down'
					print 'turtlebot is sent home and the drone lands'
					rospy.sleep(1)
					pid = 2
					pub_parr.publish(pid)
					pub_land.publish(Empty())
					rospy.sleep(1)
					pub_parr.publish(pid)
					pub_land.publish(Empty())
					sys.exit()			

			# turtlebot is finally stopped, so land			
			pub_parr.publish(pid)
			rospy.Subscriber("/ardrone/navdata", Navdata, callback_land)

			# parrot height odometry listener
			rospy.Subscriber('ardrone/odometry',Odometry,odom_listener)
			rospy.sleep(time_step)
			height_pid = z_od
			print 'landing'

		# going home siglal
		pid = 2
		counter = 0
		rospy.sleep(1)
		while counter < 1/time_step and not rospy.is_shutdown():
			counter = counter +1
			rospy.sleep(time_step)
			pub_parr.publish(pid)
			print 'going home command'
		counter = 0
		sequence_val = 4
		rospy.sleep(time_step)

	# no istruction
	else:
		pid = 0
		counter = counter +1
		pub_parr.publish(pid)
		print 100*counter/counter_limit, '%'
	
	rospy.sleep(time_step)
# end while

# forced landing drone protocol
land_force = 0
while land_force < 5:
	pub_land.publish(Empty())
	land_force = land_force +1
	rospy.sleep(time_step)
#end while forced landing drone protocol

print 'Drone disarmed'

#end main
