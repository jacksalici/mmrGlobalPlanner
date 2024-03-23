import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from mmr_base.msg import RaceStatus
from mmr_base.msg import SpeedProfilePoint, SpeedProfilePoints
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point

from .global_track import Track
from .global_trajectory import Trajectory

config = {
    "savePointsPath": False,
    "fakeDistance": 1.5,
    "legacylocalTopic": True
}

class GlobalPlanner(Node):    
    __current_lap = 0
    
    def __init__(self):
        super().__init__('race_status_sub')

        self.race_status_sub = self.create_subscription(
            RaceStatus,
            '/planning/race_status',
            self.__race_status_sub_callback,
            10)
        
        self.centerline_sub = self.create_subscription(
            Marker,
            '/planning/center_line_completed' if not config['legacylocalTopic'] else '/planning/waypoints_all',
            self.__centerline_sub_callback,
            10)
        
        self.boundaries_sub = self.create_subscription(
            Marker,
            '/planning/boundaries_all',
            self.__boundaries_sub_callback,
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
                
    def __centerline_sub_callback(self, msg: Marker):
        self.track.set_reftrack(centerline=[
            [point.x,
             point.y,
             config['fakeDistance'] if config["fakeDistance"] else point.z,
             config['fakeDistance'] if config["fakeDistance"] else point.z]
            for point in msg.points])
        self.get_logger().info(f'Saved waypoints ({len(msg.points)})')

        self.elaborateTrackline()
        self.destroy_subscription(self.centerline_sub)

    def __boundaries_sub_callback(self, msg: Marker):
        line = None

        if msg.color.r == 0 and msg.color.g == 0 and msg.color.b == 1:
            line = self.track.lines.BLUE
        elif msg.color.r == 1 and msg.color.g == 1 and msg.color.b == 0:
            line = self.track.lines.YELLOW
        else:
            self.get_logger().warning(f'Read unknown boundaries ({str(msg.color)}).')
            return
        
        self.track.add_line(points=[[point.x, point.y, point.z, point.z] for point in msg.points], line=line)

        self.get_logger().info(f'Saved boundaries [{line}] ({len(msg.points)} points).')

        if self.track.has_boundaries():
            self.destroy_subscription(self.boundaries_sub)

    def __race_status_sub_callback(self, msg: RaceStatus):
        # currentLap represent the number of laps completed. 
        if msg.current_lap != self.__current_lap:
            self.__current_lap = msg.current_lap
            self.get_logger().info(f'Lap {self.__current_lap} completed.')
            if self.__current_lap >2:
                self.destroy_subscription(self.race_status_sub)
    
    
    
    def elaborateTrackline(self):
   
        self.get_logger().info('ELABORATING TRACKLINE')
        str = self.trajectory.optimize(self.track.get_reftrack())
        
        if config['savePointsPath']:
            self.track.points_to_file(config['savePointsPath'])
        
        self.get_logger().info(f'{str}')
        
        output = self.trajectory.get_trajectory_opt()
        """
        {
            "raceline": raceline_interp,
            "speed": vx_profile_opt,
            "acceleration": ax_profile_opt,
            "heading": psi_vel_opt
        }
        """
        points_list = []
        for index, curr in enumerate(output["raceline"]):
            p = SpeedProfilePoint()
            p.point = Point(x=curr[0], y=curr[1])
            p.ackerman_point = AckermannDrive(speed=output["speed"][index], acceleration = output["acceleration"][index],steering_angle=output["heading"][index] )
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
