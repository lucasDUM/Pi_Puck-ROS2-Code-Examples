#--------Modules and Libraries--------#
import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point
from robot_features_python.getfrontier import getfrontier
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from array import *
from rclpy.node import Node # Enables the use of rclpy's Node class
from rosgraph_msgs.msg import Clock

class DetectFrontiers(Node):
    def __init__(self):
        super().__init__('df')
        
        #Sub to local map to access the map for image analysis
        self.map_subscriber = self.create_subscription(
                OccupancyGrid,
                '/epuck0/map',
                self.map_callback,
                10,
                )
        #Sub to simulated clock for time instead of using rclpy.get()Node...time etc this is wrong
        self.clock_sub = self.create_subscription (
                    Clock, 
                    '/clock',
                    self.time_callback, 
                    10)
        
      
        self.declare_parameter('goal_topic', '/epuck0/goal')
         
        (param_goal) = self.get_parameters(
            ['goal_topic'])           
        # Create publisher(s)   
        #Alternative publisher   
        #self.publish_goal_values = self.create_publisher(Float64MultiArray, '/goal', 10)
        #For visualisation:)
        self.pub = self.create_publisher(Marker, 'shapes', 10)
        self.publish_goal_values = self.create_publisher(PointStamped, param_goal.value, 10)
        
        self.goals = PointStamped()
        self.mapData = OccupancyGrid()
        self.time = Clock()
        
        self.goal_coordinates = []
        
        self.run()
        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.run)
        
    def map_callback(self, msg):
        self.mapData=msg
        
    def run(self):
        if self.mapData.header.frame_id == '' or len(self.mapData.data) < 1:
            #No map data yet
            self.get_logger().info("No map data, waiting ...")
            pass
        else:
            
            #-----------------------------------------------------#
            points = Marker()
            
            points.header.frame_id = self.mapData.header.frame_id
            #Use simulated time
            points.header.stamp = Node.get_clock(self).now().to_msg()
            
            points.ns = "markers"
            
            
            points.type = Marker.POINTS
            points.action = Marker.ADD;
            
            points.pose.orientation.w = 1.0;
            points.scale.x=points.scale.y=0.01;
            
            points.color.r = 255.0/255.0
            points.color.g = 0.0/255.0
            points.color.b = 0.0/255.0
            points.color.a = 1.0;
            
            points.lifetime == rclpy.duration.Duration();
            #-----------------------------------------------------#
            store_points = Point()
            #-----------------------------------------------------#
            
            frontiers = getfrontier(self.mapData)
            #msg = Float64MultiArray()
            #self.get_logger().info(str(type(msg.data)))
            self.goal_coordinates = []
            
            test = 0
            for i in range(len(frontiers)):
                test = test + 1
                points.id = i
                x = frontiers[i]
                #self.goal_coordinates.extend((x[0], x[1]))
                
                store_points.x = x[0]
                store_points.y = x[1]
                store_points.z = 0.0
                
                #self.get_logger().info(str(type(points.points)))
                #self.get_logger().info(str(i))
                #points.points.append(store_points)
                points.points = [store_points]
                self.pub.publish(points)
                
                
                #Pose Stamped publish
                self.goals.header.frame_id = self.mapData.header.frame_id
                self.goals.header.stamp = Node.get_clock(self).now().to_msg()
                #Node.get_clock(self).now().to_msg()
                #self.time
                
                self.goals.point.x = x[0]
                self.goals.point.y = x[1]
                self.goals.point.z = 0.0
                
                #self.get_logger().info('number = ' + str(test) + ' ' + 'Total points: ' + str(len(frontiers)) + " X, Y points: " + str(self.goals.point.x) + " " + str(self.goals.point.y))
                self.publish_goal_values.publish(self.goals)
            #self.get_logger().info(str(self.goal_coordinates))
            
            #-----------------------------------------------------#
            '''
            Alternative way of publishing goals using an array
            
            msg.data = self.goal_coordinates  
            #See this link: https://answers.ros.org/question/377979/ros2-sending-multiple-inputs-msg/
            dimension1 = MultiArrayDimension()
            dimension2 = MultiArrayDimension()
             
            dimension1.label = "data"
            dimension1.size = len(self.goal_coordinates)
            dimension1.stride = len(self.goal_coordinates) * 2
            
            dimension2.label = "goals"
            dimension2.size = 2
            dimension2.stride =  dimension1.size
            
            msg.layout.dim = [dimension1, dimension2]
            
            self.publish_goal_values.publish(msg)
            '''
            #-----------------------------------------------------#
            
    def time_callback(self, msg):
        self.time = msg.clock
                
        
def main(args=None):
    rclpy.init(args=args)
    df = DetectFrontiers()
    rclpy.spin(df)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    df.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
