import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from mmr_base.msg import RaceStatus

import json




class GlobalPlanner(Node):    
    __current_lap = 0

    def __init__(self):
        super().__init__('race_status_sub')
        self.race_status_sub = self.create_subscription(
            RaceStatus,
            '/planning/race_status',
            self.race_status_sub_callback,
            10)
        
        self.slam_cone_sub = self.create_subscription(
            Marker,
            '/slam/cones_positions',
            self.slam_cone_sub_callback,
            10)
        
        self.blue_coord = []
        self.yellow_coord = []

    def slam_cone_sub_callback(self, msg: Marker):
        # after first lap, as soon as first slam cone marker is listened, elaborate cone and destroy subs
        #first lap ends when the lap counter turns to 2.
        self.get_logger().info(f'READ {len(msg.points)}')

        if self.__current_lap == 2:
            self.slam_cone = msg
            self.elaborateConePosition()
            self.destroy_subscription(self.race_status_sub)
            self.destroy_subscription(self.slam_cone_sub)

    def race_status_sub_callback(self, msg: RaceStatus):
        # currentLap represent the number of laps completed. 
        if msg.current_lap != self.__current_lap:
            self.__current_lap = msg.current_lap
            self.get_logger().info(f'LAP {self.__current_lap} COMPLETE.')
    
    def elaborateConePosition(self):
        self.get_logger().info(f'ELABORATED {len(self.slam_cone.points)}')
        self.get_logger().info(f'ELABORATED {len(self.slam_cone.colors)}')


        for color, point in zip(self.slam_cone.colors, self.slam_cone.points):
            # Fill blue cones array
            if color.r < 0.1 and color.g < 0.1 and color.b > 0.9:
                self.blue_coord.append([point.x, point.y])
            # Fill yellow cones array
            elif color.r > 0.9 and color.g > 0.9 and color.b < 0.1:
                self.yellow_coord.append([point.x, point.y])

        with open("coords.json", "w") as f:
            json.dump({"yellow": self.yellow_coord, "blue": self.blue_coord}, f)
        
def main(args=None):
    rclpy.init(args=args)

    cone_slam_sub = GlobalPlanner()

    rclpy.spin(cone_slam_sub)

    cone_slam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
