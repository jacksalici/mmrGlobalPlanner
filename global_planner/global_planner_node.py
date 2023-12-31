import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from mmr_base.msg import RaceStatus

import numpy as np

from enum import Enum
 
class Track():
    lines = Enum('Lines', ['TRACK', 'YELLOW', 'BLUE'])
    __points = {}

    def addLine(self, line: lines, points: list):
        self.__points[line] = points

    def thereAreBoundaries(self) -> bool:
        return self.lines.YELLOW in self.__points and self.lines.BLUE in self.__points

class GlobalPlanner(Node):    
    __current_lap = 0
    
    def __init__(self):
        super().__init__('race_status_sub')
        self.get_logger().info(f'INIT GLOBAL PLANNER')

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
        
        self.track = Track()
        
        
    def waipoints_sub_callback(self, msg: Marker):
        #first lap ends when the lap counter turns to 2, then save the waypoint_all points. 
        self.track.addLine(points=msg.points, line=self.track.lines.TRACK)
        self.get_logger().info(f'SAVED WAYPOINTS ({len(msg.points)})')

        self.elaborateTrackline()

    def boundaries_sub_callback(self, msg: Marker):
        line = None

        if msg.color.r == 0 and msg.color.g == 0 and msg.color.b == 1:
            line = self.track.lines.BLUE
        elif msg.color.r == 1 and msg.color.g == 1 and msg.color.b == 0:
            line = self.track.lines.YELLOW
        else:
            self.get_logger().warning(f'READ UNKNOWN BOUNDARIES ({msg.color})')
            return

        self.track.addLine(line=line, points=msg.points)
        self.get_logger().info(f'READ BOUNDARIES [{line}] ({len(msg.points)} points)')

    def race_status_sub_callback(self, msg: RaceStatus):
        # currentLap represent the number of laps completed. 
        if msg.current_lap != self.__current_lap:
            self.__current_lap = msg.current_lap
            self.get_logger().info(f'LAP {self.__current_lap} COMPLETE.')
    
    def elaborateTrackline(self):
        self.get_logger().info('ELABORATING')


        
def main(args=None):
    rclpy.init(args=args)

    cone_slam_sub = GlobalPlanner()

    rclpy.spin(cone_slam_sub)

    cone_slam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
