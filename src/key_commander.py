#!/usr/bin/env python

# Code Done by Emanuele Sansebastiano, November 2016
import rospy

from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Int8

from getkey import getkey, keys
# if the system turn a error about getkey, use the following command in a shell to install the extension
# 'sudo pip install getkey'

rospy.init_node('key_commander', anonymous=True)

msg = """
Control The process
---------------------------

Enter one of the following keys...

q: start the turtlebot routine

a: drone take off (manual command)
s: drone land (manual command)

CTRL-C to quit
"""


#
#-------------------------------------------------------------------------------------------
#main

pub_start = rospy.Publisher("turtlebot_start", Int8, queue_size=10)
pub_takeoff = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10)
pub_land = rospy.Publisher("ardrone/land", Empty, queue_size=10)

print msg

rospy.sleep(1.0)


while not rospy.is_shutdown():

	# if the system turn a error about getkey, use the following command in a shell to install the extension
	# 'sudo pip install getkey'
	key = getkey()
	
	if key == 'q':
		pub_start.publish(1)

	elif key == 'a':
		pub_takeoff.publish(Empty())

	elif key == 's':
		pub_land.publish(Empty())

	rospy.sleep(0.1)

# to be sure the drone will land
temp = 0
while temp < 5:
	pub_land.publish(Empty())
	rospy.sleep(0.1)
	temp = temp +1

#end main



