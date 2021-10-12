#self.get_logger().info(str(cond))
#self.get_logger().info('This is z' + str(z) + 'Length of centroids' + str(len(centroids)))
#self.get_logger().info('This is z' + str(xx))

#--------Modules and Libraries--------#
import rclpy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point
from robot_features_python.getfrontier import getfrontier
from robot_features_python.utility_functions import gridValue, informationGain
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from array import *
from rclpy.node import Node # Enables the use of rclpy's Node class
from rosgraph_msgs.msg import Clock
from robot_features.msg import PointArray
from sklearn.cluster import MeanShift
from numpy import array, vstack, delete
from copy import copy
import time

class Filter(Node):
    def __init__(self):
        super().__init__('f')
        
        #Sub to local map
        self.map_subscriber = self.create_subscription(
                OccupancyGrid,
                '/epuck0/map',
                self.map_callback,
                10,
                )
        self.goal_subscriber = self.create_subscription(
                PointStamped,
                '/goal',
                self.filter_callback,
                10,
                )
        #Sub to simulated clock for time instead os uing rclpy.get()Node...time etc that isn't how you write it but hey ho
        #This is basically what the use_sim_time paramter does but was easier just hard coding it
        #Well actually i think sim_time subs to clocks and makes the rcl.py.time etc the sim time
        self.clock_sub = self.create_subscription (
                    Clock, 
                    '/clock',
                    self.time_callback, 
                    10)
        
        self.declare_parameter('goal_topic', '/epuck0/filtered_goals')
         
        (param_goal) = self.get_parameters(
            ['goal_topic']) 
            
        self.f_pub = self.create_publisher(Marker, '/frontiers', 10)
        self.c_pub = self.create_publisher(Marker, '/centroid', 10)
        #self.publish_goal_values = self.create_publisher(PointStamped, '/goal', 10)
        self.filter_pub = self.create_publisher(PointArray, param_goal.value, 10)
        
        self.goals = PointStamped()
        self.mapData = OccupancyGrid()
        self.time = Clock()
        #self.pointArray = PointArray()
        
        self.goal_coordinates = []
        self.frontiers = []
        
        self.threshold = 70
        self.info_radius = 1.0
    
    def map_callback(self, msg):
        self.mapData=msg 
        #for i in range(len(msg.data)):
            #self.get_logger().info(str(i) + ' ' + 'Map data: ' + str(msg.data[i]))
            
    def time_callback(self, msg):
        self.time = msg.clock
        
    def filter_callback(self, msg):
        '''
        This filters the points based on closeness to each other and 
        euclidean distance to the actual robot.
        This will also get rids of redendant points other time as well
        '''
        #self.get_logger().info(str(self.mapData))
        
        #Wait until a map is called
        if self.mapData.header.frame_id == '':
            return
            
            
        tempStampedPoint = PointStamped()
        tempStampedPoint.header.frame_id = self.mapData.header.frame_id
        tempStampedPoint.header.stamp = Node.get_clock(self).now().to_msg()
        tempStampedPoint.point.z = 0.0
        
        tempPoint = Point()
        tempPoint.z = 0.0
        
        pointArray = PointArray()
          
        #transformedPoint = args[0].transformPoint(args[1], data) = tf_liserner.transformPoint(Map header, data)
        #Assign frontiers from point data
        x = [array([msg.point.x, msg.point.y])]
        if len(self.frontiers) > 0:
            self.frontiers = vstack((self.frontiers, x))
        else:
            self.frontiers = x
        
        centroids = []
        front = copy(self.frontiers)
        
        #self.get_logger().info(str(len(front)))
        
        centroids = front
        #self.get_logger().info(str(x)) 
        #Is clustering needed
        
        #Unsupervised clustering technique, hierarchical clustering, figures out where and how many there are
        #This shrinks all the points about 62 to 6/7
        #https://pythonprogramming.net/hierarchical-clustering-mean-shift-machine-learning-tutorial/
        if len(front) == 1:
            centroids = front   
        elif len(front) > 1:
            #Radius(bandwidth) applied to every datapointry
            #This works
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_ # centroids array is the centers of each cluster
            
        self.frontiers = centroids
        
        #self.get_logger().info(str(len(centroids)))
#--------------------------------------------------------------------------#
#Clear old frontiers
#Check if goal is in a occupied zone
#Check tf tree, there is a mistake      
        z = 0
        while z < len(centroids):
            cond = False
            tempStampedPoint.point.x = centroids[z][0]
            tempStampedPoint.point.y = centroids[z][1]
            
            # x = transfromed points
            xx = array([tempStampedPoint.point.x, tempStampedPoint.point.y])
            #Values get here
            cond = (gridValue(self.mapData, xx) > self.threshold) or cond
            if (cond or (informationGain(self.mapData, [centroids[z][0], centroids[z][1]], self.info_radius*0.5)) < 0.2):
                centroids = delete(centroids, (z), axis=0)
                z = z-1
                
            z += 1 
#--------------------------------------------------------------------------#
#publish points
        pointArray.points = []
        num = 0
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            num = num + 1
            pointArray.points.append(copy(tempPoint))
            #self.get_logger().info('Point: ' + str(tempPoint))
        #self.get_logger().info('How many points: '+ str(num)) 
        self.filter_pub.publish(pointArray)

        
        
def main(args=None):
    rclpy.init(args=args)
    f = Filter()
    rclpy.spin(f)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    f.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
