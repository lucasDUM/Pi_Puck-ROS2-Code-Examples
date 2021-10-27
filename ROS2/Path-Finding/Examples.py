
# Author: Lucas Williamson
# Date: April 11th 2021
# ROS Version: ROS 2 Foxy Fitzroy
 
"""
Any variable names with "self" should be defined in a class
"""

#Some imports you might need
#Regular expression module
import re
from functools import partial
#--------------------------------------------------------------------------#
# Imported libraries
import numpy as np
#--------------------------------------------------------------------------#
# ROS2 client library for Python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
#--------------------------------------------------------------------------#
#Ros message types
from std_msgs.msg import String 
from geometry_msgs.msg import Twist           
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped, Point                    
#--------------------------------------------------------------------------#         
    def pose_received(self,msg):
        """
        Example of getting goal poses from frontier detection algorithm
        """
        pose_arr = msg
        goal = Point()
        dist = 0
        #arbitary big number
        curr_dist = 100
        
        #Picks goal based on euclidean distance
        for i in range(len(pose_arr.points)):
            dist_x = pose_arr.points[i].x
            dist_y = pose_arr.points[i].y
            #Math.pow is preferred over ** since it enforces float property though it is slower than **
            new_dist = math.sqrt(math.pow((self.current_x - dist_x), 2) + math.pow((self.current_y - dist_y), 2))
            self.get_logger().info('Distances: ' + str(new_dist))
            if new_dist < curr_dist and new_dist > 0.2:
                goal = pose_arr.points[i]
            curr_dist = new_dist 
            #Implement transform look up in tf-tree

        if self.new_goal:
            #Need to implement a lock so goal doesn't change until it's reached the current goal
            self.new_goal = False
            self.goal_x_coordinates = [goal.x]   
            self.goal_y_coordinates = [goal.y]
            self.goal_max_id_x = 0
            self.get_logger().info('Pose received: ' + ' x coord ' + str(self.goal_x_coordinates) + ' y coord '+ str(self.goal_y_coordinates))

#This goal would then be used by the bug2 algorithm
#Since we havce a list of goals could also implement any local path finding algorithm as long as it isn't too expensive           
#--------------------------------------------------------------------------#  
 
def laser_callback(self, msg):
        #These correspond to the Webots Driver   
        #Ranges [0] to [20] represent -150 to 150 in 15 degree increments
        #As you can imagine 0 represents the forwards direction [10]
        # -135 and 135 are the front left and right [1] and [19]
        # -90 and 90 are the sides [4] and [16]
        #-45 and 45 are the backwards directions [7] and [13] These aren't particully needed for wall following but will still be helpful
        # small ones are the short range sensors and only in forwards direction
        self.leftback_dist = msg.ranges[19] #45 back
        self.left_dist = msg.ranges[16] #90
        self.leftfront_dist = msg.ranges[13] #45 front
        self.leftfrontsmall_dist = msg.ranges[11] #small dist
        self.front_dist = msg.ranges[10]
        self.rightfrontsmall_dist = msg.ranges[9] #Small dist
        self.rightfront_dist = msg.ranges[7] #45 front 
        self.right_dist = msg.ranges[4] #90
        self.rightback_dist = msg.ranges[1] #45 back
        #Call a movement method
#--------------------------------------------------------------------------#                
def wall_bounce_random(self):
        """
        Wander around a enviroment and randomly bounce off obstacles.
        """
        # Create a Twist message and initialize all the values 
        # for the linear and angular velocities
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        """
        Bounce logic
        """
        dist = self.dist_thresh_bounce
        small_dist = 0.06
        #maybe set up cases like a switch cases but store them for use earlier
        # e.g. case[0] is set when first if statement is acheived for example
        #Might look better
        
        if   self.front_dist > dist:
            if(self.left_dist < small_dist or self.right_dist < small_dist):
                random_angle = random.randint(0, 6)         
                # Side on collision
                timer = time.time() + random_angle
                while time.time() < timer:
                    msg.angular.z = self.turning_speed
                    self.vel_publisher.publish(msg)
                msg.angular.z = 0.0
            elif(self.leftfront_dist < small_dist or self.rightfront_dist < small_dist):
                random_angle = random.randint(0, 8)         
                # Side on collision
                timer = time.time() + random_angle
                while time.time() < timer:
                    msg.angular.z = self.turning_speed
                    self.vel_publisher.publish(msg)
                msg.angular.z = 0.0
            else:
                msg.linear.x = self.forward_speed # Go straight forward
                if (self.leftfrontsmall_dist < 0.035 or self.rightfrontsmall_dist < 0.035): 
                    random_angle = random.randint(0, 12)         
                    # Head on collision
                    timer = time.time() + random_angle
                    while time.time() < timer:
                       msg.angular.z = self.turning_speed*2
                       self.vel_publisher.publish(msg)
                    msg.angular.z = 0.0          
        elif self.front_dist < dist:
            random_angle = random.randint(0, 12)         
                # Head on collision
            timer = time.time() + random_angle
            while time.time() < timer:
                msg.angular.z = self.turning_speed*2
                self.vel_publisher.publish(msg)
            msg.angular.z = 0.0
        else:
            pass
             
        self.vel_publisher.publish(msg)
#--------------------------------------------------------------------------#          
def obstacle_avoidance(self):
        # Create a Twist message and initialize all the values 
        # for the linear and angular velocities
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        
        dist = self.dist_thresh_obs
        dist_long = self.enclosed_dist_thresh_obs
        
        #Random derivation
        if   self.leftfront_dist > dist and self.front_dist > dist and self.rightfront_dist > dist:
            #front region is empty go straight
            num = random.randint(0, 100)
            self.get_logger().info(str(num) + "\n")
            if(num < 2):
                 # Head on collision
                timer = time.time() + 5
                while time.time() < timer:
                    msg.angular.z = -self.turning_speed #turn right
                    self.vel_publisher.publish(msg)
                msg.angular.z = 0.0
            elif(num < 4):
                # Head on collision
                timer = time.time() + 5
                while time.time() < timer:
                    msg.angular.z = self.turning_speed #turn left
                    self.vel_publisher.publish(msg)
                msg.angular.z = 0.0
            else:
                msg.linear.x = self.forward_speed # Go straight forward
                
                
        elif self.leftfront_dist > dist and self.front_dist < dist and self.rightfront_dist > dist or self.leftfront_dist < dist_long and self.front_dist < dist and self.rightfront_dist < dist_long:
            msg.angular.z = -self.turning_speed  # Turn left   
        elif self.leftfront_dist > dist and self.front_dist > dist and self.rightfront_dist < dist:
            #Left and front regions are free but right is taken turn left
            msg.angular.z = -self.turning_speed  
        elif self.leftfront_dist < dist and self.front_dist > dist and self.rightfront_dist > dist:
            #right and front regions are free but left is taken turn right
            msg.angular.z = -self.turning_speed # Turn right
        elif self.leftfront_dist > dist and self.front_dist < dist and self.rightfront_dist < dist:
            #Left and is free turn left
            msg.angular.z = -self.turning_speed 
        elif self.leftfront_dist < dist and self.front_dist < dist and self.rightfront_dist > dist:
             #right and is free turn right
            msg.angular.z = -self.turning_speed 
        elif self.leftfront_dist < dist and self.front_dist < dist and self.rightfront_dist < dist:
            #No where is free turn right
            msg.angular.z = -self.turning_speed 
        elif self.leftfront_dist < dist and self.front_dist > dist and self.rightfront_dist < dist:
            #Only front is free
            if self.leftfrontsmall_dist < 0.3 and self.rightfrontsmall_dist < 0.3:
                #There is a small obstacle in the way turn left
                msg.angular.z = self.turning_speed 
            else:
                msg.linear.x = self.forward_speed
        else:
            pass
             
        # Send the velocity commands to the robot by publishing 
        # to the topic
        self.vel_publisher.publish(msg)
#--------------------------------------------------------------------------#
def spin(self, msg):
        """
        Spin 360 degrees to gain more information for map merging
        """
        #This uses an Odometry reading, if too inaccurate use a time based method instead
        curr_yaw = math.ceil(self.current_yaw * (180/math.pi))
        #This is 360 degrees 
        target_yaw = 0
        
        if curr_yaw != target_yaw or self.start_spin:
            self.count_spin += 1
            if self.count_spin > 5:
                 self.start_spin = False
            self.get_logger().info('Yaw: ' + str(curr_yaw) + 'var: ' + str(self.start_spin))
            msg.angular.z = math.pi/4             
            self.vel_publisher.publish(msg)
        else:
            self.count_spin = 0
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.vel_publisher.publish(msg)
            self.get_logger().info('Finished Yaw: ' + str(curr_yaw))
            self.go_to_goal_state = "adjust heading"
#--------------------------------------------------------------------------#  