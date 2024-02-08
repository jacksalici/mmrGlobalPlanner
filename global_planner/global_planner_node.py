import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from mmr_base.msg import RaceStatus
from mmr_base.msg import SpeedProfilePoint, SpeedProfilePoints
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point

from .global_track import Track
from .global_trajectory import Trajectory
 

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
        
        self.distances_sub = self.create_subscription(
            Marker,
            '/planning/distances_all',
            self.distances_sub_callback,
            10)
        
        self.speed_profile_pub = self.create_publisher(SpeedProfilePoints, '/planning/speedProfilePoints', 10)
        
        self.track = Track(debug=True)
        self.trajectory = Trajectory()
        
        a = """Global planner initializated.

   ___ _     _         _   ___ _                        
  / __| |___| |__ __ _| | | _ | |__ _ _ _  _ _  ___ _ _ 
 | (_ | / _ | '_ / _` | | |  _| / _` | ' \| ' \/ -_| '_|
  \___|_\___|_.__\__,_|_| |_| |_\__,_|_||_|_||_\___|_|  
                                                        
"""        
        
        self.get_logger().info(a)
        
    def add_point_line(self, points, line):
        self.track.add_line(points=[[point.x, point.y] for point in points], line=line)

    def waipoints_sub_callback(self, msg: Marker):
        self.add_point_line(points=msg.points, line=self.track.lines.TRACK)
        self.get_logger().info(f'Saved waypoints ({len(msg.points)})')

        self.elaborateTrackline()
        self.destroy_subscription(self.waypoints_sub)

    def boundaries_sub_callback(self, msg: Marker):
        line = None

        if msg.color.r == 0 and msg.color.g == 0 and msg.color.b == 1:
            line = self.track.lines.BLUE
        elif msg.color.r == 1 and msg.color.g == 1 and msg.color.b == 0:
            line = self.track.lines.YELLOW
        else:
            self.get_logger().warning(f'Read unknown boundaries ({str(msg.color)}).')
            return

        self.add_point_line(line=line, points=msg.points)
        self.get_logger().info(f'Saved boundaries [{line}] ({len(msg.points)} points).')

        if self.track.has_boundaries():
            self.elaborateTrackline()
            self.destroy_subscription(self.boundaries_sub)

    def race_status_sub_callback(self, msg: RaceStatus):
        # currentLap represent the number of laps completed. 
        if msg.current_lap != self.__current_lap:
            self.__current_lap = msg.current_lap
            self.get_logger().info(f'Lap {self.__current_lap} completed.')
            if self.__current_lap >2:
                self.destroy_subscription(self.race_status_sub)
    
     
    def distances_sub_callback(self, msg: Marker):
        pass
    
    def elaborateTrackline(self):
        if self.track.is_reftrack_created() or not self.track.has_boundaries() or not self.track.has_trackline():
            return
        
        self.get_logger().info('ELABORATING TRACKLINE')
        self.track.create_reftrack()
        str = self.trajectory.optimize(self.track.get_reftrack())
        self.track.points_to_file("bag.json")
        self.get_logger().info(f'{str}')

        
        output = self.trajectory.get_trajectory_opt()
        """
        {
            "raceline": raceline_interp,
            "speed": vx_profile_opt,
            "acceleration": ax_profile_opt
        }
        """
        points_list = []
        for index, curr in enumerate(output["raceline"]):
            p = SpeedProfilePoint()
            p.point = Point(x=curr[0], y=curr[1])
            p.ackerman_point = AckermannDrive(speed=output["speed"][index], )
            points_list.append(p)
        

        self.speed_profile_pub.publish(SpeedProfilePoints(points=points_list))
        self.get_logger().info('Published.')

        

def main(args=None):
    rclpy.init(args=args)

    cone_slam_sub = GlobalPlanner()

    rclpy.spin(cone_slam_sub)

    cone_slam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
