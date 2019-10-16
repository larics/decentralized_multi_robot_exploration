#!/usr/bin/env python

#Include modules
import rospy
import time
import pandas as pd
import numpy as np
import copy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from frontier_exploration.msg import PointArray

from sklearn.cluster import AgglomerativeClustering
from collections import OrderedDict

class Filter(object):

    def __init__(self):
        # Initialize all parameters 
        # Get the frontier points
        rospy.Subscriber("/frontier_marker", MarkerArray, self.frontier_callback)
        self.frontier_pub = rospy.Publisher('filtered_frontiers', PointArray, queue_size = 1000)
        self.marker_pub = rospy.Publisher('/filtered_markers', Marker, queue_size = 1000)

        self.point_received_flag = False

        self.rateHz = rospy.get_param('~rate')
        self.map_topic = rospy.get_param('~map_topic')
        self.number_of_robots = rospy.get_param('~n_robots')        
        # The linkage distance threshold above which, clusters will not be merged
        self.distance_threshold = rospy.get_param('~distance_threshold')
        #visualization marker
        self.points_clust = Marker()
        self.points_clust.header.frame_id = self.map_topic
        self.points_clust.header.stamp = rospy.Time.now()    
        self.points_clust.type = Marker.POINTS
        self.points_clust.scale.x = 0.2
        self.points_clust.scale.y = 0.2
        self.points_clust.color.g = 1
        self.points_clust.color.a = 1
        self.points_clust.lifetime = rospy.Duration()
        
    def frontier_callback(self, marker_array):
        # Callback for the unfiltered frontier points
        self.point_received_flag = True
        self.frontier_list = [] # List of all frontier points 
        for frontier in marker_array.markers: 						
            if(len(frontier.points) > 0): 
                # Save triplets in the list								
                for triplets in frontier.points: 							
                    self.frontier_list.append([triplets.x,triplets.y,triplets.z])

    def wait_for_frontiers(self):
        # Wait if frontiers are not received yet
        while len(self.frontier_list) < 1:
            rospy.loginfo("Empty list.")
            pass

    def frontier_agglomerative_clustering(self, frontier_list):
        
        frontier_array = np.array(frontier_list)
        if frontier_array.size == 0:
            return []
        clustering = AgglomerativeClustering(affinity='euclidean', n_clusters=None, 
            compute_full_tree='True', distance_threshold=self.distance_threshold).fit(frontier_array)
        # Get the centroid of each cluster
        return self.get_centroids(frontier_array, clustering)
        
    def get_centroids(self, frontier_array, clustering):

        centroids = []
        cluster_labels = clustering.labels_
        labels_size = clustering.labels_.size
        # Group the elements from cluster_labels into tuples by index
        d = OrderedDict()
        for i, item in enumerate(cluster_labels):
            d.setdefault(item, []).append(i)
        # Get the elements of every key (label) and save them in new_list
        for key in d:
            new_list = []
            for index in d[key]:
                new_list.append(frontier_array[index])
            # Find the mean value of the each cluster (elements in new_list)
            mean = tuple(map(lambda y: sum(y) / float(len(y)), zip(*new_list)))
            centroids.append(mean)
        return centroids

    def publish_filtered_frontiers(self, centroids):
        points_array = PointArray()
        points_array.points = []
        point = Point()

        for centroid in centroids:
            point.x = centroid[0]
            point.y = centroid[1]
            point.z = 0
            points_array.points.append(copy.deepcopy(point))
        self.frontier_pub.publish(points_array)
        #print ("Points published!")
    
    def publish_filtered_markers(self, centroids):
        marker_point = Point()
        marker_array = []

        for centroid in centroids:
            marker_point.x = centroid[0]
            marker_point.y = centroid[1]
            marker_point.z = 0
            marker_array.append(copy.deepcopy(marker_point))
        self.points_clust.points = marker_array
        self.marker_pub.publish(self.points_clust)
        #print ("Markers published!")

    def run(self):
		
        rate = rospy.Rate(self.rateHz)
        while not rospy.is_shutdown():
            # Wait for markers to be received
            self.wait_for_frontiers()
            #rospy.loginfo("Waiting for frontier points.")
            
            frontier_list_main = copy.deepcopy(self.frontier_list)
            # Call filter
            centroids_main = self.frontier_agglomerative_clustering(frontier_list_main)
            print ("Number of the centroids: ", len(centroids_main))

            # Publish points and markers
            self.publish_filtered_frontiers(centroids_main)
            self.publish_filtered_markers(centroids_main)

            # When the number of filtered points < number of robots, decrease the threshold
            if len(centroids_main) < self.number_of_robots:
                rospy.loginfo("New iteration.")
                self.distance_threshold = 0.5
            else: 
                self.distance_threshold = 3.0

                #rospy.signal_shutdown("Number of filtered points less then number of mobile robots.")
            rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node("filter")

    try:
        filter = Filter()
        filter.run()
    except rospy.ROSInterruptException:
        pass

