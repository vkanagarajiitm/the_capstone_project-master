#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf.transformations as tf
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_wps=[]
	self.final_wps = []
	self.mph2mps = .447
	self.is_wps_available = False
	self.is_pos_update_available = False
	self.max_speed = 35. * self.mph2mps 
	self.min_speed = 20. * self.mph2mps 
	self.speed = 0. # start at standstill
	self.current_idx = -1
	#self.next_red_light_wp_idx = 750
	self.next_red_light_wp_idx = -1
	self.start_deceleration = False
	self.standstill = False
	#self.flag = False
        self.schedule(10)

    def schedule(self,freq):
	rate = rospy.Rate(freq)	
	#temp = 0
	while rospy.is_shutdown() != True:
	    if self.is_wps_available == True and self.is_pos_update_available == True:
	    	self.final_wps = self.wps_update()
	        self.is_pos_update_available = False
	    self.publish(self.final_wps)
	    rate.sleep()

	    #if self.flag == True:
		#temp += 1
		#if temp > 50:
		 #   self.next_red_light_wp_idx = -1

    def update_velocity_2(self,fwps):
	veh_yaw = tf.euler_from_quaternion([self.pos.orientation.x,
					  self.pos.orientation.y,
					  self.pos.orientation.z,
					  self.pos.orientation.w])
	wp_yaw = tf.euler_from_quaternion([fwps[0].pose.pose.orientation.x,
					  fwps[0].pose.pose.orientation.y,
					  fwps[0].pose.pose.orientation.z,
					  fwps[0].pose.pose.orientation.w])


	
	if math.fabs(self.pos.position.y - fwps[0].pose.pose.position.y) <= .5 and math.fabs(veh_yaw[2] - wp_yaw[2]) <= np.pi/80.:
	#self.speed += self.mph2mps
	    self.speed += 1.
	else:
	#self.speed -= self.mph2mps
	    self.speed -= 1.
	
	if self.speed > self.max_speed:
	    self.speed = self.max_speed
	if self.speed < self.min_speed:
	    self.speed = self.min_speed

	for wp in fwps:
	    wp.twist.twist.linear.x = self.speed
	return fwps
    def update_velocity(self,fwps):
	if self.next_red_light_wp_idx > 0:
	    dist = 1000000
	    dist = self.distance(self.base_wps,self.current_idx,self.next_red_light_wp_idx)
	    if dist <= (self.speed*1.5):
		if self.start_deceleration == False:
		    self.max_speed = self.speed
		self.start_deceleration = True
	else:
	    self.start_deceleration = False
	
	if self.start_deceleration == True:
	    #self.speed -= self.mph2mps	    
	    self.speed -= .5
	    if self.speed <= -1.:
		#self.flag = True
	        self.speed = -1.
		self.standstill = True
	    for wp in fwps:
	        wp.twist.twist.linear.x = self.speed
        elif self.standstill == True:
            #veh_yaw = tf.euler_from_quaternion([self.pos.orientation.x,
		#			  self.pos.orientation.y,
		#			  self.pos.orientation.z,
		#			  self.pos.orientation.w])
	    #wp_yaw = tf.euler_from_quaternion([fwps[0].pose.pose.orientation.x,
		#			  fwps[0].pose.pose.orientation.y,
		#			  fwps[0].pose.pose.orientation.z,
		#			  fwps[0].pose.pose.orientation.w])

	    #self.max_speed = fwps[0].twist.twist.linear.x
	    self.speed += 1.
	    print(self.max_speed)
	    #self.min_speed = self.max_speed - 3.
	
	    #if math.fabs(self.pos.position.y - fwps[0].pose.pose.position.y) <= 1. and math.fabs(veh_yaw[2] - wp_yaw[2]) <= np.pi/40.:
	        #self.speed += self.mph2mps
		#self.speed += 1.
	    #else:
	        #self.speed -= self.mph2mps
		#self.speed -= 1.
	
	    if self.speed >= self.max_speed:
	        self.speed = self.max_speed
		self.standstill == False
	    #if self.speed < self.min_speed:
	    #    self.speed = self.min_speed

	    for wp in fwps:
	        wp.twist.twist.linear.x = self.speed
	else:
	    self.speed = fwps[0].twist.twist.linear.x

	#print("speed = " + str(fwps[0].twist.twist.linear.x))

	return fwps

    def wps_update(self):
        final_wps = []
	#print(((self.pos.position.x)))
	#is_point_located = False
	#if self.pos.position.x < 100.:
	#    self.current_idx = 0
	if self.current_idx < 0:
	    for idx in range(len(self.base_wps)):
	        if(self.pos.position.x >= self.base_wps[(self.current_idx+idx)%len(self.base_wps)].pose.pose.position.x and
		       self.pos.position.x <= self.base_wps[(self.current_idx+idx+1)%len(self.base_wps)].pose.pose.position.x) or (
			(self.pos.position.x <= self.base_wps[(self.current_idx+idx)%len(self.base_wps)].pose.pose.position.x and
		        self.pos.position.x >= self.base_wps[(self.current_idx+idx+1)%len(self.base_wps)].pose.pose.position.x)) and (
			math.fabs(self.pos.position.y - self.base_wps[(idx)%len(self.base_wps)].pose.pose.position.y) <= 1. or (
			math.fabs(self.pos.position.y - self.base_wps[(idx+1)%len(self.base_wps)].pose.pose.position.y) <= 1.)):
		    self.current_idx = idx + 1
                    break
	else:
	    for idx in range(self.current_idx,self.current_idx + 200):
		if(self.pos.position.x >= self.base_wps[(idx)%len(self.base_wps)].pose.pose.position.x and
		       self.pos.position.x <= self.base_wps[(idx+1)%len(self.base_wps)].pose.pose.position.x) or (
			(self.pos.position.x <= self.base_wps[(idx)%len(self.base_wps)].pose.pose.position.x and
		        self.pos.position.x >= self.base_wps[(idx+1)%len(self.base_wps)].pose.pose.position.x)):
		    self.current_idx = (idx + 1)%len(self.base_wps)
		    #print(self.current_idx)
		    #print(self.base_wps[self.current_idx].pose.pose.position.x)
	            break

        for idx in range(0,LOOKAHEAD_WPS):
	    #temp = self.base_wps[(idx + self.current_idx)%len(self.base_wps)]
	    #temp.twist.twist.linear.x = self.max_speed
            final_wps.append(self.base_wps[(idx + self.current_idx)%len(self.base_wps)])
	    #final_wps.append(temp)

	if len(final_wps) > 0:
	    final_wps = self.update_velocity(final_wps)
	#print("wp = " + str(final_wps[0].pose.pose.position.x))
	#print(self.current_idx)
	return final_wps

    def pose_cb(self, msg):
        # TODO: Implement
	self.is_pos_update_available = True
        self.pos = msg.pose

	#self.final_wps = self.wps_update()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_wps = waypoints.waypoints
	self.is_wps_available = True
       
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
	self.next_red_light_wp_idx = msg.data
        #print(msg.data)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish(self,waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
