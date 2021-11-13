#!/usr/bin/env python
from std_msgs.msg import Int64
import rospy
from geometry_msgs.msg import Twist
from time import sleep

t = Twist()

def buttons():
    print "w,a,s,d controlls for forward back left right:"
    print "make shure you hit enter after each controller use and keyborad is lower case:"
    print "p to exit program:"


def callback(data):
	rospy.loginfo("NODE MOVE %s",data.data)

#and not rospy.is_shutdown(): 

def moveForward(speed,time):
	if speed > 0.33:
		return
	print 'IN moveforward' 

	t_start = rospy.Time.now()
	
	dur = rospy.Duration(time)
	r = rospy.Rate(30)
	pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=1)
	while (rospy.Time.now() - t_start) < dur and not rospy.is_shutdown():
		t.linear.x = speed
		pub.publish(t)
		r.sleep()
	t.linear.x = 0
	pub.publish(t)

def moveBackwards(speed,time):
	print 'IN moveBackwards'
	if speed > 0.33:
		return 

	t_start = rospy.Time.now()
	
	dur = rospy.Duration(time)
	r = rospy.Rate(30)
	pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=1)
	while (rospy.Time.now() - t_start) < dur and not rospy.is_shutdown():
		t.linear.x = -speed
		pub.publish(t)
		r.sleep()
	t.linear.x = 0
	pub.publish(t)

def turnRight(speed,time):
	if speed > 0.52:
		return
	print 'IN turnRight' 

	t_start = rospy.Time.now()
	
	dur = rospy.Duration(time)
	r = rospy.Rate(30)
	pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=1)
	while (rospy.Time.now() - t_start) < dur and not rospy.is_shutdown():
		t.angular.z = -speed
		pub.publish(t)
		r.sleep()
	t.angular.z = 0
	pub.publish(t)

def turnLeft(speed,time):
	if speed > 0.52:
		return
	print 'IN turnRight' 

	t_start = rospy.Time.now()
	
	dur = rospy.Duration(time)
	r = rospy.Rate(30)
	pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=1)
	while (rospy.Time.now() - t_start) < dur and not rospy.is_shutdown():
		t.angular.z = speed
		pub.publish(t)
		r.sleep()
	t.angular.z = 0
	pub.publish(t)

def main():
	rospy.init_node('node_move')
	
	quit = False

	rate = rospy.Rate(10)
    
   buttons()
   speed = input("what is the velocity: ")

	while quit != True:
		key = str(raw_input())
		if key == "w":
			moveForward(speed,2)
			sleep(2)
		if key == "s":
			moveBackwards(speed,2)
			sleep(2)
		if key == "d":
			turnRight(speed,6.3)
			sleep(2)
		if key == "a":
			turnLeft(speed,6.3)
			sleep(2)
       if key == "p":
           quit = True
           break
            

	while not rospy.is_shutdown():
		#print("Trying to move")
		pub.publish(t)
		rate.sleep()

main()
