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

TOF_SENSOR_ANGLE = [
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

        
        
        self.tof_sensors = {}
        for i in range(TOF_SENSORS):
            sensor_tof = self.robot.getDistanceSensor('tof{}'.format(i))
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

    def __publish_laserscan_data(self):
    
        #get the namespace for the robots
        namespace = rclpy.node.Node.get_namespace(self)
        #self.get_logger().info('Namespace of robot X  = "%s"' % namespace)
        tf = namespace + "/"
        
        stamp = Time(seconds=self.robot.getTime()).to_msg()
        dists = [OUT_OF_RANGE] * NB_INFRARED_SENSORS
        dist_tofs = [OUT_OF_RANGE] * TOF_SENSORS
        dist_tof = OUT_OF_RANGE

        # Calculate distances
        for i, key in enumerate(self.distance_sensors):
            dists[i] = interpolate_lookup_table(
                self.distance_sensors[key].getValue(), self.distance_sensors[key].getLookupTable()
            )
            
        # Publish range: ToFs
        for i, key in enumerate(self.tof_sensors):
            dist_tofs[i] = interpolate_lookup_table(
                self.tof_sensors[key].getValue(), self.tof_sensors[key].getLookupTable()
            )

        # Publish range: ToF
        if self.tof_sensor:
            dist_tof = interpolate_lookup_table(self.tof_sensor.getValue(), self.tof_sensor.getLookupTable())

        # Max range of ToF sensor is 1.35m so we put it as maximum laser range.
        # Therefore, for all invalid ranges we put 0 so it get deleted by rviz
        laser_dists = [OUT_OF_RANGE if dist > INFRARED_MAX_RANGE else dist for dist in dists]
        tof_dists = [OUT_OF_RANGE if dist > TOF_SHORT_MODE_MAX_RANGE else dist for dist in dist_tofs]
        
        msg = LaserScan()
        msg.header.frame_id = (tf + 'laser_scanner').replace("/", "")
        msg.header.stamp = stamp
        msg.angle_min = - 150 * pi / 180
        msg.angle_max = 150 * pi / 180
        msg.angle_increment = 15 * pi / 180
        msg.range_min = INFRARED_MIN_RANGE + SENSOR_DIST_FROM_CENTER
        msg.range_max = TOF_SHORT_MODE_MAX_RANGE + SENSOR_DIST_FROM_CENTER
        
        
        #Set up an if statement to change the ranges in the future 
        
        msg.ranges = [
            laser_dists[3] + SENSOR_DIST_FROM_CENTER,   # -150
            tof_dists[2] + SENSOR_DIST_FROM_CENTER,     # -135 tof
            OUT_OF_RANGE,                               # -120
            OUT_OF_RANGE,                               # -105
            tof_dists[3] + SENSOR_DIST_FROM_CENTER,     # -90 tof ~changes when low
            OUT_OF_RANGE,                               # -75
            OUT_OF_RANGE,                               # -60
            tof_dists[4] + SENSOR_DIST_FROM_CENTER,     # -45 tof ~changes when low
            OUT_OF_RANGE,                               # -30
            laser_dists[0],   # -15
            dist_tof + SENSOR_DIST_FROM_CENTER,         # 0
            laser_dists[7],   # 15
            OUT_OF_RANGE,                               # 30
            tof_dists[5] + SENSOR_DIST_FROM_CENTER,     # 45 tof ~changes when low
            OUT_OF_RANGE,                               # 60
            OUT_OF_RANGE,                               # 75
            tof_dists[0] + SENSOR_DIST_FROM_CENTER,     # 90 tof ~changes when low
            OUT_OF_RANGE,                               # 105
            OUT_OF_RANGE,                               # 120
            tof_dists[1] + SENSOR_DIST_FROM_CENTER,     # 135 tof
            laser_dists[4] + SENSOR_DIST_FROM_CENTER,   # 150
        ]
        self.laser_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckDriver(args=args)

    rclpy.spin(epuck_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
