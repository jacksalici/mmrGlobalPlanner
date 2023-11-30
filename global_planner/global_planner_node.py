import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from mmr_base.msg import RaceStatus





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
        
    
    def slam_cone_sub_callback(self, msg: Marker):
        # after first lap, as soon as first slam cone marker is listened, elaborate cone and destroy subs
        if self.__current_lap == 1:
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
        self.get_logger().info(f'ELABORATED {str(len(self.slam_cone.points))} CONES.')

        
def main(args=None):
    rclpy.init(args=args)

    cone_slam_sub = GlobalPlanner()

    rclpy.spin(cone_slam_sub)

    cone_slam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
