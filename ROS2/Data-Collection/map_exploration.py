# Author: Lucas Williamson
# Date: April 11th 2021
# ROS Version: ROS 2 Foxy Fitzroy
 
############## IMPORT LIBRARIES #################
# Python math library
import math 
# Python time library
import time

import re
# import the math module 
import math

# ROS client library for Python
import rclpy 
 
# Enables pauses in the execution of code
from time import sleep 
 
# Used to create nodes
from rclpy.node import Node
 
# Enables the use of the string message type
from std_msgs.msg import String 
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist     
                     
# Handles LaserScan messages to sense distance to obstacles (i.e. walls)        
from sensor_msgs.msg import LaserScan    
 
# Handle Pose messages
from geometry_msgs.msg import Pose 
 
#Handle map info
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray
                     
# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_action_status_default
 
# Scientific computing library
import numpy as np

#from nav_msgs.msg import Odometry 

from functools import partial

class MapExplore(Node):
    def __init__(self):
        super().__init__('me')
        name = "/epuck"
        map_name = ''
        odom_name = ''
        map_subscriber = []
        odom_subscriber = []
        self.num_robots = 5
        self.number = 0
        
        #Add a param to automate file creation for new iterations
        #This method is based on using a ground truth through the /map topic
        
        self.odom_file = open("odom_data.txt", "w+")
        self.map_completion_file = open("map_completion_data.txt", "w+")
        #self.map_efficiency_file = open("map_efficency_data.txt", "w+")
        #self.map_quality_file = open("map_quality_data.txt", "w+")
        
        #To compare with
        self.truth_map = OccupancyGrid()
        self.truth_map_filled_num = 0
        self.truth_map_free_num = 0
        
        #Checks 5 pixels either side of it for a match
        self.error_margin = 5
        
        self.map_subscriber = self.create_subscription(
                OccupancyGrid,
                '/map',
                self.truth_map_callback,
                qos_profile=qos_profile_action_status_default,
                )
        self.map_subscriber = self.create_subscription(
                OccupancyGrid,
                '/map_merge/map',
                self.merge_map_callback,
                10,
                )
                
        #This is for indivisual robot stats  
        for number in range (self.num_robots):
            self.number = number
            odom_name = name + str(number) + '/odom'
            self.odom_subscriber = self.create_subscription(
                Odometry,
                odom_name,
                partial(self.odom_callback, odom_name),
                10,
                )
            
        self.total_distance = 0.0
        self.left_dist = 0.0
        self.previous_x = [0.0] * self.num_robots
        self.previous_y = [0.0] * self.num_robots
        self.first_run = [True] * self.num_robots
        
    def merge_map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height 
        size = len(msg.data)
        
        num_instances_of_negatives = 0
        num_instances_of_positives = 0
        num_instances_of_zeros = 0
        other = 0
        matches = 0
        map_completion = 0
        map_quality = 0
        
        for i in range(int(size)):
            number = msg.data[i]
            if(i>4 and i < (len(self.truth_map.data)-3)):
            	ground_truth = [0]*self.error_margin
            	for margin in range(self.error_margin):
            	    ground_truth[margin] =  self.truth_map.data[(i-2+margin)]
            if(number == -1):
                num_instances_of_negatives += 1
            elif(number == 0):
                num_instances_of_zeros += 1
                if(number == ground_truth[0] or number == ground_truth[1] or number == ground_truth[2] or number == ground_truth[3] or number == ground_truth[4]):
                    matches = matches + 1
            elif(number == 100):
                num_instances_of_positives +=1
                if(number == ground_truth[0] or number == ground_truth[1] or number == ground_truth[2] or number == ground_truth[3] or number == ground_truth[4]):
                    matches = matches + 1
            else:
                other +=1
        #ratio lets make it a percentage
        map_completion = ((num_instances_of_zeros + num_instances_of_positives)/(self.truth_map_filled_num + self.truth_map_free_num)) * 100
        self.map_completion_file.write(str(map_completion) + " ")
        
         #ratio lets make it a percentage
        map_quality = matches/(self.truth_map_filled_num + self.truth_map_free_num) * 100
        self.map_quality_file.write(str(map_quality) + " ")
        
        
    def truth_map_callback(self, msg):
        self.truth_map = msg
        
        width = msg.info.width
        height = msg.info.height 
        size = len(msg.data)
        
        num_instances_of_negatives = 0
        num_instances_of_positives = 0
        num_instances_of_zeros = 0
        other = 0
        for i in range(int(size)):
            number = msg.data[i]
            if(number == -1):
                num_instances_of_negatives += 1
            elif(number == 0):
                num_instances_of_zeros += 1
            elif(number == 100):
                num_instances_of_positives +=1
            else:
                other +=1
        self.truth_map_filled_num =  num_instances_of_positives
        self.truth_map_free_num = num_instances_of_zeros+20 
             
    def odom_callback(self, name, msg):
        number = [int(s) for s in re.findall(r'-?\d+\.?\d*', name)]
        #self.get_logger().info(str(number) + name)
        if(self.first_run[number[0]]):
            self.previous_x[number[0]] = msg.pose.pose.position.x
            self.previous_y[number[0]] = msg.pose.pose.position.y
      
            
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
      
        d_increment = math.sqrt((x - self.previous_x[number[0]]) * (x - self.previous_x[number[0]]) + (y - self.previous_y[number[0]]) * (y - self.previous_y[number[0]]))
        
        self.total_distance = self.total_distance + d_increment
        #self.pub.publish(data)
        self.previous_x[number[0]] = msg.pose.pose.position.x
        self.previous_y[number[0]] = msg.pose.pose.position.y
        self.first_run[number[0]] = False
        
        
       
        
        #self.get_logger().info("Total distance traveled is {:.2f} m".format(self.total_distance))
        
        self.total_distance = self.total_distance + d_increment
        self.odom_file.write(str(self.total_distance) + " ")
        #self.get_logger().info("before: " + str(x) + " " + str(self.get_topic_name()))
        #self.get_logger().info("\n After: " + str(self.previous_x[number[0]]) + " " + str(self.previous_y[number[0]]))
        #self.get_logger().info("\n Maths: " + str( x - self.previous_x))
        
            
def main(args=None):
    rclpy.init(args=args)
    me = MapExplore()
    rclpy.spin(me)

    me.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
