# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 e-puck driver."""

from math import pi
import rclpy
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from webots_ros2_core.math.interpolation import interpolate_lookup_table
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode


OUT_OF_RANGE = 0.0
INFRARED_MAX_RANGE = 0.04
INFRARED_MIN_RANGE = 0.009
TOF_MAX_RANGE = 1.0
TOF_SHORT_MODE_MIN_RANGE = 0.04
TOF_SHORT_MODE_MAX_RANGE = 1.35
TOF_SENSORS = 6
DEFAULT_WHEEL_RADIUS = 0.02
DEFAULT_WHEEL_DISTANCE = 0.05685
NB_INFRARED_SENSORS = 8
SENSOR_DIST_FROM_CENTER = 0.035


DISTANCE_SENSOR_ANGLE = [
    -15 * pi / 180,   # ps0
    -45 * pi / 180,   # ps1
    -90 * pi / 180,   # ps2
    -150 * pi / 180,  # ps3
    150 * pi / 180,   # ps4
    90 * pi / 180,    # ps5
    45 * pi / 180,    # ps6
    15 * pi / 180,    # ps7
]

TOF_LIDAR_SENSOR_ANGLE = [
    90 * pi / 180,    # tof0 absolute angle 0
    135  * pi / 180,  # tof1 absolute angle 45
    -135 * pi / 180,  # tof2 absolute angle 135
    -90 * pi / 180,   # tof3 absolute angle 180
    -45 * pi / 180,   # tof4 absolute angle 225
    45 * pi / 180,    # tof5 absolute angle 315
]


DEVICE_CONFIG = {
    'camera': {'topic_name': ''},
    'robot': {'publish_base_footprint': True},
    'ps0': {'always_publish': True},
    'ps1': {'always_publish': True},
    'ps2': {'always_publish': True},
    'ps3': {'always_publish': True},
    'ps4': {'always_publish': True},
    'ps5': {'always_publish': True},
    'ps6': {'always_publish': True},
    'ps7': {'always_publish': True},
    'tof': {'always_publish': True},
    'tof0': {'always_publish': True},
    'tof1': {'always_publish': True},
    'tof2': {'always_publish': True},
    'tof3': {'always_publish': True},
    'tof4': {'always_publish': True},
    'tof5': {'always_publish': True}
}


class EPuckDriver(WebotsDifferentialDriveNode):
    def __init__(self, args):
        super().__init__(
            'robot',
            args,
            wheel_distance=DEFAULT_WHEEL_DISTANCE,
            wheel_radius=DEFAULT_WHEEL_RADIUS
        )
        self.start_device_manager(DEVICE_CONFIG)
        
        #get the namespace for the robots
        namespace = rclpy.node.Node.get_namespace(self)
        #self.get_logger().info('Namespace of robot X  = "%s"' % namespace)
        
        # Intialize distance sensors for LaserScan topic
        self.distance_sensors = {}
        for i in range(NB_INFRARED_SENSORS):
            sensor = self.robot.getDistanceSensor('ps{}'.format(i))
            sensor.enable(self.timestep)
            self.distance_sensors['ps{}'.format(i)] = sensor

        
        # Create Lidar subscriber
        #self.lidar_sensor = self.robot.getLidar('lidar_sensor')
        #self.lidar_sensor.enable(self.service_node_vel_timestep)
        #self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)
        
        
        self.tof_sensors = {}
        for i in range(TOF_SENSORS):
            sensor_tof = self.robot.getLidar('tof{}'.format(i))
            sensor_tof.enable(self.timestep)
            self.tof_sensors['tof{}'.format(i)] = sensor_tof
        
        scan_frame = namespace + '/scan'
        self.laser_publisher = self.create_publisher(LaserScan, scan_frame, 1)
        
        
        self.tof_sensor = self.robot.getDistanceSensor('tof')
        if self.tof_sensor:
            self.tof_sensor.enable(self.timestep)
        else:
            self.get_logger().info('ToF sensor is not present for this e-puck version')
        
        # Main loop
        self.create_timer(self.timestep / 1000, self.__publish_laserscan_data)
        
    #def null_values(num):
        #nulls = []
        #for i in range(num):
            #nulls [i] = OUT_OF_RANGE
        #return nulls
        
    def __publish_laserscan_data(self):
    
        #get the namespace for the robots
        namespace = rclpy.node.Node.get_namespace(self)
        #self.get_logger().info('Namespace of robot X  = "%s"' % namespace)
        tf = namespace + "/"
        
        stamp = Time(seconds=self.robot.getTime()).to_msg()
        dists = [OUT_OF_RANGE] * NB_INFRARED_SENSORS
        dist_tofs = [OUT_OF_RANGE] * TOF_SENSORS
        dist_tof = OUT_OF_RANGE
        tofs_dists = [OUT_OF_RANGE] * TOF_SENSORS
        
        test_tofs = [OUT_OF_RANGE] * 13

        # Calculate distances
        for i, key in enumerate(self.distance_sensors):
            dists[i] = interpolate_lookup_table(
                self.distance_sensors[key].getValue(), self.distance_sensors[key].getLookupTable()
            )
            
        # Publish range: ToF
        if self.tof_sensor:
            dist_tof = interpolate_lookup_table(self.tof_sensor.getValue(), self.tof_sensor.getLookupTable())

        # Max range of ToF sensor is 1.35m so we put it as maximum laser range.
        # Therefore, for all invalid ranges we put 0 so it get deleted by rviz
        laser_dists = [OUT_OF_RANGE if dist > INFRARED_MAX_RANGE else dist for dist in dists]
        #tof_dists = [OUT_OF_RANGE if dist > TOF_SHORT_MODE_MAX_RANGE else dist for dist in dist_tofs]
        
        #  msg_lidar.ranges = self.lidar_sensor.getRangeImage()
        
        #Create data array for ranges
        data = [OUT_OF_RANGE] * 1501 # An array with 1501 datapoints defaulting to 0
        
        #It was quickest to manually set the data
        #You should automate this process
        
        #Lasers
        data[0] = laser_dists[3] + SENSOR_DIST_FROM_CENTER
        data[1500] = laser_dists[4] + SENSOR_DIST_FROM_CENTER
        data[675] = laser_dists[0] + SENSOR_DIST_FROM_CENTER
        data[825] = laser_dists[7] + SENSOR_DIST_FROM_CENTER
        data[750] = dist_tof + SENSOR_DIST_FROM_CENTER
        #The gap is 2.0 so we have a little bit of error between
        #Tof0
        data[1150] = self.tof_sensors['tof0'].getRangeImage()[0] + SENSOR_DIST_FROM_CENTER#80
        data[1158] = self.tof_sensors['tof0'].getRangeImage()[1] + SENSOR_DIST_FROM_CENTER
        data[1166] = self.tof_sensors['tof0'].getRangeImage()[2] + SENSOR_DIST_FROM_CENTER
        data[1174] = self.tof_sensors['tof0'].getRangeImage()[3] + SENSOR_DIST_FROM_CENTER
        data[1182] = self.tof_sensors['tof0'].getRangeImage()[4] + SENSOR_DIST_FROM_CENTER
        data[1190] = self.tof_sensors['tof0'].getRangeImage()[5] + SENSOR_DIST_FROM_CENTER
        data[1198] = self.tof_sensors['tof0'].getRangeImage()[6] + SENSOR_DIST_FROM_CENTER
        data[1206] = self.tof_sensors['tof0'].getRangeImage()[7] + SENSOR_DIST_FROM_CENTER
        data[1214] = self.tof_sensors['tof0'].getRangeImage()[8] + SENSOR_DIST_FROM_CENTER 
        data[1222] = self.tof_sensors['tof0'].getRangeImage()[9] + SENSOR_DIST_FROM_CENTER
        data[1230] = self.tof_sensors['tof0'].getRangeImage()[10] + SENSOR_DIST_FROM_CENTER
        data[1238] = self.tof_sensors['tof0'].getRangeImage()[11] + SENSOR_DIST_FROM_CENTER
        data[1250] = self.tof_sensors['tof0'].getRangeImage()[12] + SENSOR_DIST_FROM_CENTER#100 slight error propagated 0.2 too off
        #Tof1
        data[1375] = self.tof_sensors['tof1'].getRangeImage()[0] + SENSOR_DIST_FROM_CENTER#125
        data[1383] = self.tof_sensors['tof1'].getRangeImage()[1] + SENSOR_DIST_FROM_CENTER
        data[1391] = self.tof_sensors['tof1'].getRangeImage()[2] + SENSOR_DIST_FROM_CENTER
        data[1399] = self.tof_sensors['tof1'].getRangeImage()[3] + SENSOR_DIST_FROM_CENTER
        data[1407] = self.tof_sensors['tof1'].getRangeImage()[4] + SENSOR_DIST_FROM_CENTER
        data[1415] = self.tof_sensors['tof1'].getRangeImage()[5] + SENSOR_DIST_FROM_CENTER
        data[1423] = self.tof_sensors['tof1'].getRangeImage()[6] + SENSOR_DIST_FROM_CENTER
        data[1431] = self.tof_sensors['tof1'].getRangeImage()[7] + SENSOR_DIST_FROM_CENTER
        data[1439] = self.tof_sensors['tof1'].getRangeImage()[8] + SENSOR_DIST_FROM_CENTER
        data[1447] = self.tof_sensors['tof1'].getRangeImage()[9] + SENSOR_DIST_FROM_CENTER
        data[1455] = self.tof_sensors['tof1'].getRangeImage()[10] + SENSOR_DIST_FROM_CENTER 
        data[1463] = self.tof_sensors['tof1'].getRangeImage()[11] + SENSOR_DIST_FROM_CENTER
        data[1475] = self.tof_sensors['tof1'].getRangeImage()[12] + SENSOR_DIST_FROM_CENTER#145 slight error propagated 0.2 too off
        
        #Tof2
        data[25] = self.tof_sensors['tof2'].getRangeImage()[0] + SENSOR_DIST_FROM_CENTER#-145
        data[33] = self.tof_sensors['tof2'].getRangeImage()[1] + SENSOR_DIST_FROM_CENTER
        data[41] = self.tof_sensors['tof2'].getRangeImage()[2] + SENSOR_DIST_FROM_CENTER
        data[49] = self.tof_sensors['tof2'].getRangeImage()[3] + SENSOR_DIST_FROM_CENTER
        data[57] = self.tof_sensors['tof2'].getRangeImage()[4] + SENSOR_DIST_FROM_CENTER
        data[65] = self.tof_sensors['tof2'].getRangeImage()[5] + SENSOR_DIST_FROM_CENTER
        data[73] = self.tof_sensors['tof2'].getRangeImage()[6] + SENSOR_DIST_FROM_CENTER
        data[81] = self.tof_sensors['tof2'].getRangeImage()[7] + SENSOR_DIST_FROM_CENTER
        data[89] = self.tof_sensors['tof2'].getRangeImage()[8] + SENSOR_DIST_FROM_CENTER
        data[97] = self.tof_sensors['tof2'].getRangeImage()[9] + SENSOR_DIST_FROM_CENTER
        data[105] = self.tof_sensors['tof2'].getRangeImage()[10] + SENSOR_DIST_FROM_CENTER 
        data[113] = self.tof_sensors['tof2'].getRangeImage()[11] + SENSOR_DIST_FROM_CENTER
        data[125] = self.tof_sensors['tof2'].getRangeImage()[12] + SENSOR_DIST_FROM_CENTER#-125 slight error propagated 0.8 degreetoo off
       
        #Tof3
        data[250] = self.tof_sensors['tof3'].getRangeImage()[0] + SENSOR_DIST_FROM_CENTER#-100
        data[258] = self.tof_sensors['tof3'].getRangeImage()[1] + SENSOR_DIST_FROM_CENTER
        data[266] = self.tof_sensors['tof3'].getRangeImage()[2] + SENSOR_DIST_FROM_CENTER
        data[274] = self.tof_sensors['tof3'].getRangeImage()[3] + SENSOR_DIST_FROM_CENTER
        data[282] = self.tof_sensors['tof3'].getRangeImage()[4] + SENSOR_DIST_FROM_CENTER
        data[290] = self.tof_sensors['tof3'].getRangeImage()[5] + SENSOR_DIST_FROM_CENTER
        data[298] = self.tof_sensors['tof3'].getRangeImage()[6] + SENSOR_DIST_FROM_CENTER
        data[306] = self.tof_sensors['tof3'].getRangeImage()[7] + SENSOR_DIST_FROM_CENTER
        data[314] = self.tof_sensors['tof3'].getRangeImage()[8] + SENSOR_DIST_FROM_CENTER
        data[322] = self.tof_sensors['tof3'].getRangeImage()[9] + SENSOR_DIST_FROM_CENTER
        data[330] = self.tof_sensors['tof3'].getRangeImage()[10] + SENSOR_DIST_FROM_CENTER 
        data[338] = self.tof_sensors['tof3'].getRangeImage()[11] + SENSOR_DIST_FROM_CENTER
        data[350] = self.tof_sensors['tof3'].getRangeImage()[12] + SENSOR_DIST_FROM_CENTER#-80 slight error propagated 0.8 degreetoo off
        
        #Tof4
        data[475] = self.tof_sensors['tof4'].getRangeImage()[0] + SENSOR_DIST_FROM_CENTER#-55
        data[483] = self.tof_sensors['tof4'].getRangeImage()[1] + SENSOR_DIST_FROM_CENTER
        data[491] = self.tof_sensors['tof4'].getRangeImage()[2] + SENSOR_DIST_FROM_CENTER
        data[499] = self.tof_sensors['tof4'].getRangeImage()[3] + SENSOR_DIST_FROM_CENTER
        data[507] = self.tof_sensors['tof4'].getRangeImage()[4] + SENSOR_DIST_FROM_CENTER
        data[515] = self.tof_sensors['tof4'].getRangeImage()[5] + SENSOR_DIST_FROM_CENTER
        data[523] = self.tof_sensors['tof4'].getRangeImage()[6] + SENSOR_DIST_FROM_CENTER
        data[531] = self.tof_sensors['tof4'].getRangeImage()[7] + SENSOR_DIST_FROM_CENTER
        data[539] = self.tof_sensors['tof4'].getRangeImage()[8] + SENSOR_DIST_FROM_CENTER
        data[547] = self.tof_sensors['tof4'].getRangeImage()[9] + SENSOR_DIST_FROM_CENTER
        data[555] = self.tof_sensors['tof4'].getRangeImage()[10] + SENSOR_DIST_FROM_CENTER 
        data[563] = self.tof_sensors['tof4'].getRangeImage()[11] + SENSOR_DIST_FROM_CENTER
        data[575] = self.tof_sensors['tof4'].getRangeImage()[12] + SENSOR_DIST_FROM_CENTER#-35 slight error propagated 0.8 degreetoo off
        
        #Tof5
        data[925] = self.tof_sensors['tof5'].getRangeImage()[0] + SENSOR_DIST_FROM_CENTER# 35
        data[933] = self.tof_sensors['tof5'].getRangeImage()[1] + SENSOR_DIST_FROM_CENTER
        data[941] = self.tof_sensors['tof5'].getRangeImage()[2] + SENSOR_DIST_FROM_CENTER
        data[949] = self.tof_sensors['tof5'].getRangeImage()[3] + SENSOR_DIST_FROM_CENTER
        data[957] = self.tof_sensors['tof5'].getRangeImage()[4] + SENSOR_DIST_FROM_CENTER
        data[965] = self.tof_sensors['tof5'].getRangeImage()[5] + SENSOR_DIST_FROM_CENTER
        data[973] = self.tof_sensors['tof5'].getRangeImage()[6] + SENSOR_DIST_FROM_CENTER
        data[981] = self.tof_sensors['tof5'].getRangeImage()[7] + SENSOR_DIST_FROM_CENTER
        data[989] = self.tof_sensors['tof5'].getRangeImage()[8] + SENSOR_DIST_FROM_CENTER
        data[997] = self.tof_sensors['tof5'].getRangeImage()[9] + SENSOR_DIST_FROM_CENTER
        data[1005] = self.tof_sensors['tof5'].getRangeImage()[10] + SENSOR_DIST_FROM_CENTER 
        data[1013] = self.tof_sensors['tof5'].getRangeImage()[11] + SENSOR_DIST_FROM_CENTER
        data[1025] = self.tof_sensors['tof5'].getRangeImage()[12] + SENSOR_DIST_FROM_CENTER#55 slight error propagated 0.8 degreetoo off
         
        #tof_dists = [OUT_OF_RANGE if dist > TOF_SHORT_MODE_MAX_RANGE else dist for dist in data]
        
        msg = LaserScan()
        msg.header.frame_id = (tf + 'laser_scanner').replace("/", "")
        msg.header.stamp = stamp
        msg.angle_min = - 150 * pi / 180
        msg.angle_max = 150 * pi / 180
        msg.angle_increment = 0.2 * pi / 180 
        msg.range_min = SENSOR_DIST_FROM_CENTER + INFRARED_MIN_RANGE
        msg.range_max = SENSOR_DIST_FROM_CENTER + TOF_SHORT_MODE_MAX_RANGE
        msg.ranges = data
        self.laser_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckDriver(args=args)

    rclpy.spin(epuck_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
