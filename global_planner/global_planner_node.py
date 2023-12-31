import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from mmr_base.msg import RaceStatus

import numpy as np

from enum import Enum
 
class Track():
    lines = Enum('Lines', ['TRACK', 'YELLOW', 'BLUE'])
    __points = {}

    def addLine(self, type: lines, points: list):
        self.__points[type] = points

    def thereAreBoundaries(self) -> bool:
        return self.lines.YELLOW in self.__points and self.lines.BLUE in self.__points

class GlobalPlanner(Node):    
    __current_lap = 0
    
    def __init__(self):
        super().__init__('race_status_sub')
        self.race_status_sub = self.create_subscription(
            RaceStatus,
            '/planning/race_status',
            self.race_status_sub_callback,
            10)
        
        self.waypoints_sub = self.create_subscription(
            Marker,
            '/planning/waypoints_all',
            self.waipoints_sub_callback,
            10)
        
        self.boundaries_sub = self.create_subscription(
            Marker,
            '/planning/boundaries_all',
            self.boundaries_sub_callback,
            10)
        
    def waipoints_sub_callback(self, msg: Marker):
        # after first lap, as soon as first slam cone marker is listened, elaborate waypoints and destroy subs
        #first lap ends when the lap counter turns to 2.
        self.get_logger().info(f'READ {len(msg.points)}')

        if self.__current_lap == 2:
            self.waypoints = msg
            self.elaborateTrackline()
            self.destroy_subscription(self.race_status_sub)
            self.destroy_subscription(self.waypoints_sub)


    def race_status_sub_callback(self, msg: RaceStatus):
        # currentLap represent the number of laps completed. 
        if msg.current_lap != self.__current_lap:
            self.__current_lap = msg.current_lap
            self.get_logger().info(f'LAP {self.__current_lap} COMPLETE.')
    
    def elaborateTrackline(self):
        self.get_logger().info(f'ELABORATED {len(self.waypoints.points)}')
        self.get_logger().info(f'ELABORATED {len(self.waypoints.colors)}')


        
def main(args=None):
    rclpy.init(args=args)

    cone_slam_sub = GlobalPlanner()

    rclpy.spin(cone_slam_sub)

    cone_slam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
