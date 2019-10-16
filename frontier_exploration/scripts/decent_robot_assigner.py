#!/usr/bin/env python
import rospy
import sys
import tf
import time
import numpy as np
import rosservice
from copy import copy
from math import exp, hypot
from hungarian import Hungarian
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from functions import robot, informationGain
from frontier_exploration.msg import PointArray, RobotPosGoal
from frontier_exploration.srv import RobotService

class RobotAssigner(object):

	def __init__(self):
		# Initialization
		self.map_topic = rospy.get_param('~map_topic', '/map')
		self.info_radius = rospy.get_param('~info_radius', 0.2)							
		self.frontiers_topic = rospy.get_param('~frontiers_topic','/filtered_frontiers')
		self.n_robots = rospy.get_param('~n_robots')
		self.delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.2)
		self.rateHz = rospy.get_param('~rate', 1)
		self.robot_number = rospy.get_param('~robot_number')
		self.rate = rospy.Rate(self.rateHz)

		self.frontiers = PointArray()		
		# Subscribers 
		rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
		rospy.Subscriber(self.frontiers_topic, PointArray, self.frontier_callback)
	
		# Initialization
		self.mapData = OccupancyGrid()
		self.move_base_error_point = {}
		# Current goals for all robots
		self.current_goals = {}
		
		self.wait_for_map()
		self.wait_for_frontiers()
		 
		# Get the name of the robot and creat robots
		self.append_robot_name()
		# Rosservice definition and wait for availability
		self.mission_service = rospy.Service('startmission' +
			 str(self.robot_number), RobotService, self.handleStartmission)
		
		self.wait_for_all_services()
		print ("Services are ready!")

	def map_callback(self, data):
		self.mapData = data

	def frontier_callback(self, data):
		# PointArray
		self.frontiers = data

	def wait_for_frontiers(self):
		# Wait if frontiers are not received yet
		while len(self.frontiers.points) < self.n_robots:
			pass

	def wait_for_map(self):
		# Wait if map is not received yet
		print ("Waiting for map")
		while len(self.mapData.data) < 1:
			pass
		print ("Map received.")

	def append_robot_name(self):
		self.robots = []
		for i in range (self.n_robots):
			self.robots.append(robot('/robot_' + str(i)))
		self.robots[self.robot_number].sendGoal(
				self.robots[self.robot_number].getPosition())

	def wait_for_all_services(self):
		print("Waiting for services...")
		for i in range (0, self.n_robots):
			rospy.wait_for_service('startmission' + str(i))
		
	def start_new_mission(self, robot_id):
		# Call the service_robot_id to get robot(robot_id) position
		service_response = RobotPosGoal()
		try:				
			start_mission = rospy.ServiceProxy(
				'startmission' + str(robot_id), RobotService)
			service_response = start_mission(robot_id)
			return service_response
		except rospy.ServiceException as exc:
			rospy.logwarn("Service did not process request: " + str(exc))	

	def handleStartmission(self, request):
		# request --> robot_id int32 type
		# response --> position Point() type
		response = RobotPosGoal()
		response.position = self.robots[request.robot_id].getPosition()
		response.current_goal = self.robots[request.robot_id].current_goal
		return response

	def calculate_weight_matrix(self, frontier_points):
		# Calculate the weight for every frontier point	and for all robots
		# Create matrix of weights (n x m)
		number_of_frontiers = len(frontier_points.points)
		weight_matrix = np.zeros((self.n_robots, number_of_frontiers))
						
		list_of_utilities = self.get_list_of_utilities(frontier_points)
		for i in range(0, self.n_robots):
			# Call the services to get position and current_goal of the robot
			response = self.start_new_mission(i)
			list_of_sum_costs_and_frontier_occ = \
				self.get_list_of_sum_costs_and_frontier_occ(frontier_points, response.response)
			# Fill the current_goals dict
			self.current_goals[i] = response.response.current_goal
			# The weight = cost - utility + frontier_occ
			weight_matrix[i] =[a - b \
				for a, b in zip(list_of_sum_costs_and_frontier_occ, list_of_utilities)]
			# Set all negative values to zero
			weight_matrix[i] = [0 if x < 0 else x for x in weight_matrix[i]]
		return weight_matrix
		
	def calculate_cost(self, frontier_point, robot_position):
		# Cost function for a single point = Euclidean distance
		lambda_c = 1.5
		cost = lambda_c * self.Euclidean_distance(
			frontier_point, robot_position)
		return cost

	def calculate_utility(self, frontier_point):
		# Utility function for a single point
		lambda_u = 2.0
		utility = lambda_u * (informationGain(self.mapData, 
			[frontier_point.x, frontier_point.y], self.info_radius))
		return utility

	def calculate_frontier_occupancy(self, frontier_point, current_goal):
		# Calculate frontier_occupancy_function for other mobile robots
		# And only if the frontier point is in range radius_front_occ from current_goal
		radius_front_occ = 8.0
		lambda_f = 20.0
		# distance = distance from frontier_point to current_goal
		distance = self.Euclidean_distance(frontier_point, current_goal)
		if (distance <= radius_front_occ):
			first_factor = \
				((frontier_point.x - current_goal.x) ** 2) / (2 * ((radius_front_occ) ** 2))
			second_factor = \
				((frontier_point.y - current_goal.y) ** 2) / (2 * ((radius_front_occ) ** 2))
			
			F = lambda_f * (exp(-(first_factor + second_factor)))
		else:
			F = 0
		return F
	
	def get_list_of_utilities(self, frontier_points):
		# Collect utilities of all frontier points
		list_of_utilities = []
		for point in frontier_points.points:
			list_of_utilities.append(self.calculate_utility(point))
		return list_of_utilities

	def get_list_of_sum_costs_and_frontier_occ(self, frontier_points, response):
		list_of_costs = []
		list_of_frontier_occ = []
		
		for point in frontier_points.points:
			# Cost function			
			list_of_costs.append(
				self.calculate_cost(point, response.position))
			# Frontier occupancy function
			list_of_frontier_occ.append(
				self.calculate_frontier_occupancy(point, response.current_goal))
		# Return the sum of these two lists
		list_of_sum_costs_and_frontier_occ = [a + b \
			for a, b in zip(list_of_costs, list_of_frontier_occ)]
		return list_of_sum_costs_and_frontier_occ

	def calculate_Hungarian(self, weight_matrix):
		# Find the minimum using Hungarian algorithm
		
		# If number of robots > number of frontiers
		# If the number of raws > number of columns in weight_matrix
		# if len(weight_matrix) > len(weight_matrix[0]): add!!
		
		# Check if the weight matrix is complete	
		if len(weight_matrix) == self.n_robots:
			h = Hungarian(weight_matrix)
			h.calculate()
			result = h.get_results()
		else:
			result = []
			rospy.logwarn("Invalid input for Hungarian algorithm.")
		print ('result je', result)
		
		return result

	def remove_points(self, mutable_centroids):
		# Remove current_goals and error points form constant_frontiers	
		constant_frontiers = PointArray()
		if len(self.current_goals) > 1 or len(self.move_base_error_point) > 0:
			current_goals_list = [*self.current_goals.values()]
			error_point_list = [*self.move_base_error_point.values()]
			# Save the points in constant frontiers
			for point in mutable_centroids.points:
				if not (self.check_if_point_in_list(current_goals_list, point) or \
					self.check_if_point_in_list(error_point_list, point)):							
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
	
	def Euclidean_distance(self, first_point, second_point):
		return hypot(first_point.x - second_point.x, 
			first_point.y - second_point.y)

	def run(self):

		rospy.loginfo("Robot assigner started")

		while not rospy.is_shutdown():
			self.wait_for_frontiers()
			mutable_centroids = copy(self.frontiers)
		
			# Remove current_goals and error points form constant_frontiers	
			constant_frontiers = self.remove_points(mutable_centroids)
			
			# Calculate weight matrix
			weight_matrix = self.calculate_weight_matrix(constant_frontiers)
			# Calculate Hungarian, result --> (robot_index, goal)
			hungarian_results = self.calculate_Hungarian(weight_matrix)

			robot_result = [item \
				for item in hungarian_results if item[0] == self.robot_number]			
			robot_index, goal_index = robot_result[0]				
			point_to_send = constant_frontiers.points[goal_index]
			# Send robot to the goal
			self.robots[self.robot_number].sendGoal(point_to_send)
			rospy.loginfo("Robot_" + str(self.robot_number) +
				 "  assigned to point  " + str(point_to_send))
			rospy.sleep(self.delay_after_assignement)
			
									
			start_position = self.robots[self.robot_number].getPosition()
			goal_position = constant_frontiers.points[goal_index]							
			start_time = time.time()
			start_time_timer = time.time()
			# Move until goal is reached, less that 1 meter
			while not (self.Euclidean_distance(
				start_position, goal_position)) < 1.0:
				
				# If the goal is aborted
				if (self.robots[self.robot_number].getState() == 4):
					print ("STATE", self.robots[self.robot_number].getState())
					# Rememeber the point to delete in the next iteration
					self.move_base_error_point[robot_index] = goal_position
					break

				# If a robot is in the same position for 5 seconds	
				current_position = self.robots[self.robot_number].getPosition()
				if (self.Euclidean_distance(
					start_position, current_position) < 0.2 and 
					(time.time() - start_time) > 4.0): 
					# Robot in the same position for 4 seconds
					print ('Robot ', self.robot_number, 'is not moving.')
					# Rememeber the point to delete in the next iteration
					self.move_base_error_point[robot_index] = goal_position
					break
				else:
					start_time = time.time()
					start_position = self.robots[self.robot_number].getPosition()

				# Generally, if the robot needs more than 10 seconds to 
				# reach the goal, delete that goal
				if (time.time() - start_time_timer) > 60.0:
					self.move_base_error_point[robot_index] = goal_position
					print ("MORE THAN 60 SECONDS!")
					break
				rospy.sleep(1.0)
			print ('Robot ', self.robot_number, 'reached the goal')		
			self.rate.sleep()


			
if __name__ == '__main__':
	rospy.init_node('Robot_assigner')
	try:
		robot_assigner = RobotAssigner()
		robot_assigner.run()
	except rospy.ROSInterruptException:
		pass




		