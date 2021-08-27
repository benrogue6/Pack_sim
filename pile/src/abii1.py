#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Laser_act(): #laser scan class
   
    def __init__(self): #main function
        global movee
        global obs_distance
        global inf_distance
        movee = Twist() #create object of twist type  
        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #publish message
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback) #subscribe message 
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) #subscribe message
        

    def laser_callback(self, msg): #function for obstacle avoidance
	"""
        front_regions = {
          'front': msg.ranges[0], #lidar data for front side
          'front-l': msg.ranges[22], 
          'front-r': msg.ranges[338], 
	}
        left_regions = {
          'left': msg.ranges[90], #lidar data for left side
          'left-up': msg.ranges[67], 
          'left-down': msg.ranges[113], 
	}
	right_regions = {
          'right': msg.ranges[270], #lidar data for right side
          'right-up': msg.ranges[293], 
          'right-down': msg.ranges[247], 
	}
	back_regions = {
          'back-r': msg.ranges[203], #lidar data for back side
          'back-l': msg.ranges[157], 
          'back': msg.ranges[180], 
	}
	"""
        print '-------RECEIVING LIDAR SENSOR DATA-------'
        print 'Front: {}'.format(msg.ranges[0]) #lidar data for front side
        print 'front-l: {}'.format(msg.ranges[22])
        print 'front-r:  {}'.format(msg.ranges[338]) 
	print 'ft-lt:  {}'.format(msg.ranges[315]) 
        print 'left: {}'.format(msg.ranges[90]) #lidar data for left side
        print 'left-up: {}'.format(msg.ranges[67])
        print 'left-down: {}'.format(msg.ranges[113])
        print 'Right: {}'.format(msg.ranges[270]) #lidar data for right side
        print 'right-up: {}'.format(msg.ranges[293])
	print 'right-down: {}'.format(msg.ranges[247])
	print 'Back: {}'.format(msg.ranges[180]) #lidar data for back side
        print 'back-l: {}'.format(msg.ranges[157])
        print 'back-r:  {}'.format(msg.ranges[203])

	self.front_obstacle(msg)

 
    def front_obstacle(self, msg):
	movee = Twist()
	linear_x = 0.2
	angular_z = 0.0
	  
	obs_distance = 0.4
        inf_distance = 0.5

	if msg.ranges[0] > obs_distance and msg.ranges[22] > obs_distance and msg.ranges[338] > obs_distance:
            movee.linear.x = 0.3 
            movee.angular.z = 0.0 
            rospy.loginfo("no-obstacle")
	
	if msg.ranges[0] > obs_distance and msg.ranges[22] > obs_distance and msg.ranges[338] < obs_distance:
            movee.linear.x = 0.2 
            movee.angular.z = 0.5 
            rospy.loginfo("right-obstacle")

	if msg.ranges[0] > obs_distance and msg.ranges[22] < obs_distance and msg.ranges[338] > obs_distance:
            movee.linear.x = 0.2 
            movee.angular.z = -0.5 
            rospy.loginfo("left-obstacle") 

	if msg.ranges[0] < obs_distance and msg.ranges[22] > obs_distance and msg.ranges[338] > obs_distance:
            movee.linear.x = 0.0
            movee.angular.z = 0.5 
            rospy.loginfo("front-obstacle")

	if msg.ranges[0] < obs_distance and msg.ranges[22] > obs_distance and msg.ranges[338] < obs_distance:
            movee.linear.x = 0.0 
            movee.angular.z = 0.5 
            rospy.loginfo("front-r-obstacle")

	if msg.ranges[0] < obs_distance and msg.ranges[22] < obs_distance and msg.ranges[338] > obs_distance:
            movee.linear.x = 0.0 
            movee.angular.z = -0.5 
            rospy.loginfo("front-r-obstacle")  
	
 	"""
	if msg.ranges[0] > obs_distance and msg.ranges[22] < obs_distance and msg.ranges[338] > obs_distance:
            movee.linear.x = 0.2 
            movee.angular.z = 0.5 
            rospy.loginfo("left-obstacle") 
		
	if msg.ranges[0] < obs_distance and msg.ranges[22] < obs_distance and msg.ranges[338] > obs_distance:
            movee.linear.x = 0.2 
            movee.angular.z = 0.5 
            rospy.loginfo("front-l-obstacle") 
	if msg.ranges[0] < obs_distance and msg.ranges[22] > obs_distance and msg.ranges[338] < obs_distance:
            movee.linear.x = 0.2 
            movee.angular.z = -0.3 
            rospy.loginfo("front-r-obstacle") 
	if msg.ranges[0] > obs_distance and msg.ranges[22] < obs_distance and msg.ranges[338] < obs_distance:
            movee.linear.x = 0.3 
            movee.angular.z = 0.0 
            rospy.loginfo("right-left-obstacle")
	if msg.ranges[0] < obs_distance and msg.ranges[22] < obs_distance and msg.ranges[338] < obs_distance:
            movee.linear.x = -0.2 
            movee.angular.z = 0.2 
            rospy.loginfo("fr-l-r-obstacle")  
	"""

	print movee.linear.x
	print movee.linear.z
        self.cmd_vel_pub.publish(movee) # publish the move object  
     
    def odometry(self, msg): #function for odometry
	pass
        #print msg.pose.pose #print position and orientation of turtlebot


if __name__ == '__main__':
    rospy.init_node('dodge_track') #initialize node

    laser_act = Laser_act() # laser_act object
    
    rospy.spin() #loop it#!/usr/bin/env python


