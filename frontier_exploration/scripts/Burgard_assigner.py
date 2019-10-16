#!/usr/bin/env python
import rospy
import sys
import tf
import numpy as np
import rosservice
from copy import copy
import time
from numpy import inf
from math import exp, hypot
from numpy import array
from numpy.linalg import norm
from hungarian import Hungarian
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from frontier_exploration.msg import PointArray, FrontierTF
from functions import robot, informationGain

class RobotAssigner(object):

	def __init__(self):

		self.map_topic = rospy.get_param('~map_topic', '/map')
		self.info_radius = rospy.get_param('~info_radius', 2.0)					#this can be smaller than the laser scanner range,                                              					#be accurate
		self.info_multiplier = rospy.get_param('~info_multiplier', 3.0)				#constant, weight (information_gain and cost do not have the same units
		self.hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)			#at least as much as the laser scanner range
		self.hysteresis_gain = rospy.get_param('~hysteresis_gain', 2.0)				#bigger than 1 (biase robot to continue exploring current region
		self.frontiers_topic = rospy.get_param('~frontiers_topic','/filtered_frontiers')
		#self.robot_info = rospy.get_param('~robot_info','/robot_information')
		self.n_robots = rospy.get_param('~n_robots')
		self.delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
		self.rateHz = rospy.get_param('~rate', 100)
		#self.robot_number = rospy.get_param('~robot_number')
		self.rate = rospy.Rate(self.rateHz)

		self.frontiers = PointArray()
		self.assigned_point = []	#last point assigned to each robot
		#self.error_point = {new_list: [] for new_list in range(self.n_robots)}
		self.error_point = []

		self.there_is_a_robot_available = False
		
		# Subscribers
		rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
		rospy.Subscriber(self.frontiers_topic, PointArray, self.frontier_callback)
		# Publisher
		self.pub = rospy.Publisher("/assigned_point", Marker, queue_size=2)
		self.pub_point = Point()
		self.mapData = OccupancyGrid()
		self.is_once_assigned = False
		# Define marker
		self.marker = Marker()		
		self.marker.type = 8
		self.marker.header.frame_id = self.map_topic
		self.marker.scale.x = 0.5
		self.marker.scale.y = 0.5
		self.marker.color.b = 0.3 * 1
		self.marker.color.a = 0.3 * (1 + 1)

		self.wait_for_map()
		self.wait_for_frontiers()
		 
		#get the name of the robot and creat robots
		self.append_robot_name()

	def map_callback(self, data):
		self.mapData = data

	def frontier_callback(self, data):
		# PointArray
		self.frontiers = data

	def wait_for_frontiers(self):
		# Wait if frontiers are not received yet
		while len(self.frontiers.points) < 1:
			pass
		#print("Frontiers received.")
	
	def wait_for_map(self):
		# Wait if map is not received yet
		print("Waiting for map.")
		while len(self.mapData.data) < 1:
			pass
		print("Map received.")

	def append_robot_name(self):
		self.robots = []
		for i in range(self.n_robots):
			self.robots.append(robot('/robot_' + str(i)))

		for i in range(0, self.n_robots):
			self.robots[i].sendGoal(self.robots[i].getPosition())
		
	def point_in_array(self, arr, point):
		for x in arr:
			if x == point:
				return True
		return False

	def add_point_to_error_list(self):
		if not self.point_in_array(self.error_point, self.max_point.frontier_point):
			self.error_point.append(self.max_point.frontier_point)
			self.there_is_a_robot_available = True

	def remove_points(self, mutable_centroids):
		# Remove assigned_points and error points form mutable_frontiers	
		constant_frontiers = PointArray()

		# Save the points in constant frontiers
		for point in mutable_centroids.points:
			if len(self.assigned_point) > 0 or len(self.error_point) > 0:
				if not (self.check_if_point_in_list(self.assigned_point, point) or \
					self.check_if_point_in_list(self.error_point, point)):							
					constant_frontiers.points.append(point)				
			else:
				for point in mutable_centroids.points:
					constant_frontiers.points.append(point)
		
		return constant_frontiers

	def check_if_point_in_list(self, list_of_points, point):
		for point_in_list in list_of_points:
			if (point.x == point_in_list.x and \
				point.y == point_in_list.y):
				return True
		return False 

	def run(self):
		start = time.time()
		rospy.loginfo("Robot assigner started")
	
		watch_dogs_list = []
		for i in range (0, self.n_robots):
			watch_dogs_list.append(WatchDog(self.robots[i]))
				
		while not rospy.is_shutdown():
		
			mutable_centroids = copy(self.frontiers)
			self.wait_for_frontiers()
			# Get number of available/busy robots
			for i in range(0, self.n_robots):
				state = self.robots[i].getState()
				if not (state == 2 or \
					state == 1 or \
						state == 0):
						# Set a flag
						print ("Stanje mi je", state, " i uci cu u pelju kaze", i)
						self.there_is_a_robot_available = True
	
			constant_frontiers = self.remove_points(mutable_centroids)
			# Initialize cells
			list_of_cells = []
			for ip in range(0, len(constant_frontiers.points)):
				list_of_cells.append(Cell(constant_frontiers.points[ip]))
			#print ("State robot 1", self.robots[1].getState())

			# Find the revenue and send the robot to the goal
			if self.there_is_a_robot_available:
				print("Racunam novu tocku")
				if len(list_of_cells) > 0:
					for i in range(0, self.n_robots):
						self.max_point = list_of_cells[0]
						for point in list_of_cells:
							point_revenue = point.calculate_cost_and_utility(self.robots[i])
							if point_revenue > self.max_point.get_revenue():
								if not (point.is_reserved):
									self.max_point = point
						if not (self.max_point.is_reserved):	
							self.robots[i].sendGoal(self.max_point.frontier_point)
							self.max_point.is_reserved = True
						else:
							self.there_is_a_robot_available = True	

						self.is_once_assigned = True
						self.assigned_point.append(self.max_point.frontier_point)
						watch_dogs_list[i].set_goal_and_timer(self.max_point.frontier_point)

						rospy.loginfo("Robot_" + str(i) + "  assigned to  " + str(self.max_point.frontier_point))	
						rospy.sleep(self.delay_after_assignement)
					self.there_is_a_robot_available = False
				else:
					print ("nema celija")

			if (self.is_once_assigned):
				for i in range(0, self.n_robots):
					watchdog = watch_dogs_list[i]
					if (watchdog.check_if_not_moving()):
						# More than 4 sec and less than 0.4 m						
						print("Robot_" + str(i) + " is not moving.")
						self.add_point_to_error_list()

					if (watch_dogs_list[i].check_if_time_elapsed()):
						# More than 7 sec
						print ("Too much time for robot:", i)
						self.add_point_to_error_list()

					if self.robots[i].getState() == 4:
						print("Abort")
						# Check if the point is already in the list
						self.add_point_to_error_list()	
	
			self.rate.sleep()

class Cell():

	cost = 0.0
	distance = 0.0

	def __init__(self, frontier_point):
		self.frontier_point = frontier_point
		self.utility = 1.0
		self.beta = 1.0
		self.d_max = 20.0 #sensor range
		self.is_reserved = False

	def get_revenue(self):
		revenue = self.utility - self.beta * self.cost
		return revenue

	def calculate_cost_and_utility(self, robot):
		robot_position = robot.getPosition()
		distance = hypot(robot_position.x - self.frontier_point.x,
			robot_position.y - self.frontier_point.y)
		self.cost = 2 * distance
		self.utility = self.utility - (1.0 - (distance / self.d_max))
		return self.get_revenue()

class WatchDog():

	def __init__(self, robot):
		self.robot = robot
		self.start_position = robot.getPosition()
		self.max_waiting_time = 20.0
		self.max_not_moving_time = 7.0
		self.position_tolerance = 0.4
	
	def start(self):
		self.start_time = time.time()
		
	def check_if_time_elapsed(self):
		#If timer is greater than 7.0 seconds
		if (time.time() - self.start_time) > self.max_waiting_time:
			return True
		return False

	def check_if_not_moving(self):
		current_position = self.robot.getPosition() 
		if (time.time() - self.start_time) > self.max_not_moving_time and \
			hypot(current_position.x - self.start_position.x, 
			current_position.y - self.start_position.y) < self.position_tolerance:
			return True
		return False	

	def set_goal_and_timer(self, goal_point):
		self.goal_point = goal_point
		self.reset()
		self.start()

	def reset(self):
		self.start_time = time.time()
	
	def check_all(self, current_position):
		if (self.check_if_time_elapsed() or self.check_if_not_moving(current_position)):
			return True
		return False


if __name__ == '__main__':
	rospy.init_node('Robot_assigner_Burgard')
	try:
		robot_assigner = RobotAssigner()
		robot_assigner.run()
	except rospy.ROSInterruptException:
		pass
