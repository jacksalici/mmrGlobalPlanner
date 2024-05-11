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
        self.params_dict = self.get_params()
        


        self.race_status_sub = self.create_subscription(
            RaceStatus,
            '/planning/race_status',
            self.__race_status_sub_callback,
            10)
        
        self.centerline_sub = self.create_subscription(
            Marker,
            '/planning/center_line_completed' if not self.params_dict['misc']['legacylocalTopic'] else '/planning/waypoints_all',
            self.__centerline_sub_callback,
            10)
        
        self.boundaries_sub = self.create_subscription(
            Marker,
            '/planning/boundaries_all',
            self.__boundaries_sub_callback,
            10)
    
        self.speed_profile_pub = self.create_publisher(SpeedProfilePoints, '/planning/speedProfilePoints', 10)

        self.track = Track(debug=self.params_dict['misc']['debug'])
        self.trajectory = Trajectory(params = self.params_dict)
        
        a = """Global planner initializated.

   ___ _     _         _   ___ _                        
  / __| |___| |__ __ _| | | _ | |__ _ _ _  _ _  ___ _ _ 
 | (_ | / _ | '_ / _` | | |  _| / _` | ' \| ' \/ -_| '_|
  \___|_\___|_.__\__,_|_| |_| |_\__,_|_||_|_||_\___|_|  
                                                        
"""        
        
        
        
        self.get_logger().info(a)


    def get_params(self):
        
        params_dict = {}
       # generated params loader for ROS
        params_dict['car_config'] = {}
        self.declare_parameter('car_config.ggv_file')
        params_dict['car_config']['ggv_file'] = self.get_parameter('car_config.ggv_file').value
        self.declare_parameter('car_config.ax_max_machines_file')
        params_dict['car_config']['ax_max_machines_file'] = self.get_parameter('car_config.ax_max_machines_file').value
        params_dict['car_config']['stepsize_opts'] = {}
        self.declare_parameter('car_config.stepsize_opts.stepsize_prep')
        params_dict['car_config']['stepsize_opts']['stepsize_prep'] = self.get_parameter('car_config.stepsize_opts.stepsize_prep').value
        self.declare_parameter('car_config.stepsize_opts.stepsize_reg')
        params_dict['car_config']['stepsize_opts']['stepsize_reg'] = self.get_parameter('car_config.stepsize_opts.stepsize_reg').value
        self.declare_parameter('car_config.stepsize_opts.stepsize_interp_after_opt')
        params_dict['car_config']['stepsize_opts']['stepsize_interp_after_opt'] = self.get_parameter('car_config.stepsize_opts.stepsize_interp_after_opt').value
        params_dict['car_config']['reg_smooth_opts'] = {}
        self.declare_parameter('car_config.reg_smooth_opts.k_reg')
        params_dict['car_config']['reg_smooth_opts']['k_reg'] = self.get_parameter('car_config.reg_smooth_opts.k_reg').value
        self.declare_parameter('car_config.reg_smooth_opts.s_reg')
        params_dict['car_config']['reg_smooth_opts']['s_reg'] = self.get_parameter('car_config.reg_smooth_opts.s_reg').value
        params_dict['car_config']['veh_params'] = {}
        self.declare_parameter('car_config.veh_params.v_max')
        params_dict['car_config']['veh_params']['v_max'] = self.get_parameter('car_config.veh_params.v_max').value
        self.declare_parameter('car_config.veh_params.length')
        params_dict['car_config']['veh_params']['length'] = self.get_parameter('car_config.veh_params.length').value
        self.declare_parameter('car_config.veh_params.width')
        params_dict['car_config']['veh_params']['width'] = self.get_parameter('car_config.veh_params.width').value
        self.declare_parameter('car_config.veh_params.mass')
        params_dict['car_config']['veh_params']['mass'] = self.get_parameter('car_config.veh_params.mass').value
        self.declare_parameter('car_config.veh_params.dragcoeff')
        params_dict['car_config']['veh_params']['dragcoeff'] = self.get_parameter('car_config.veh_params.dragcoeff').value
        self.declare_parameter('car_config.veh_params.curvlim')
        params_dict['car_config']['veh_params']['curvlim'] = self.get_parameter('car_config.veh_params.curvlim').value
        self.declare_parameter('car_config.veh_params.g')
        params_dict['car_config']['veh_params']['g'] = self.get_parameter('car_config.veh_params.g').value
        params_dict['car_config']['vel_calc_opts'] = {}
        self.declare_parameter('car_config.vel_calc_opts.dyn_model_exp')
        params_dict['car_config']['vel_calc_opts']['dyn_model_exp'] = self.get_parameter('car_config.vel_calc_opts.dyn_model_exp').value
        self.declare_parameter('car_config.vel_calc_opts.vel_profile_conv_filt_window')
        params_dict['car_config']['vel_calc_opts']['vel_profile_conv_filt_window'] = self.get_parameter('car_config.vel_calc_opts.vel_profile_conv_filt_window').value
        params_dict['optimization_opt'] = {}
        params_dict['optimization_opt']['optim_opts_mincurv'] = {}
        self.declare_parameter('optimization_opt.optim_opts_mincurv.width_opt')
        params_dict['optimization_opt']['optim_opts_mincurv']['width_opt'] = self.get_parameter('optimization_opt.optim_opts_mincurv.width_opt').value
        self.declare_parameter('optimization_opt.optim_opts_mincurv.iqp_iters_min')
        params_dict['optimization_opt']['optim_opts_mincurv']['iqp_iters_min'] = self.get_parameter('optimization_opt.optim_opts_mincurv.iqp_iters_min').value
        self.declare_parameter('optimization_opt.optim_opts_mincurv.iqp_curverror_allowed')
        params_dict['optimization_opt']['optim_opts_mincurv']['iqp_curverror_allowed'] = self.get_parameter('optimization_opt.optim_opts_mincurv.iqp_curverror_allowed').value
        params_dict['misc'] = {}
        self.declare_parameter('misc.savePointsPath')
        params_dict['misc']['savePointsPath'] = self.get_parameter('misc.savePointsPath').value
        self.declare_parameter('misc.fakeDistance')
        params_dict['misc']['fakeDistance'] = self.get_parameter('misc.fakeDistance').value
        self.declare_parameter('misc.legacylocalTopic')
        params_dict['misc']['legacylocalTopic'] = self.get_parameter('misc.legacylocalTopic').value
        self.declare_parameter('misc.debug')
        params_dict['misc']['debug'] = self.get_parameter('misc.debug').value
        params_dict['imp_opts'] = {}
        self.declare_parameter('imp_opts.min_track_width')
        params_dict['imp_opts']['min_track_width'] = self.get_parameter('imp_opts.min_track_width').value
        params_dict['lap_time_mat_opts'] = {}
        self.declare_parameter('lap_time_mat_opts.use_lap_time_mat')
        params_dict['lap_time_mat_opts']['use_lap_time_mat'] = self.get_parameter('lap_time_mat_opts.use_lap_time_mat').value
        self.declare_parameter('lap_time_mat_opts.gg_scale_range')
        params_dict['lap_time_mat_opts']['gg_scale_range'] = self.get_parameter('lap_time_mat_opts.gg_scale_range').value
        self.declare_parameter('lap_time_mat_opts.gg_scale_stepsize')
        params_dict['lap_time_mat_opts']['gg_scale_stepsize'] = self.get_parameter('lap_time_mat_opts.gg_scale_stepsize').value
        self.declare_parameter('lap_time_mat_opts.top_speed_range')
        params_dict['lap_time_mat_opts']['top_speed_range'] = self.get_parameter('lap_time_mat_opts.top_speed_range').value
        self.declare_parameter('lap_time_mat_opts.top_speed_stepsize')
        params_dict['lap_time_mat_opts']['top_speed_stepsize'] = self.get_parameter('lap_time_mat_opts.top_speed_stepsize').value
        self.declare_parameter('lap_time_mat_opts.file')
        params_dict['lap_time_mat_opts']['file'] = self.get_parameter('lap_time_mat_opts.file').value
        params_dict['plot_opts'] = {}
        self.declare_parameter('plot_opts.mincurv_curv_lin')
        params_dict['plot_opts']['mincurv_curv_lin'] = self.get_parameter('plot_opts.mincurv_curv_lin').value
        self.declare_parameter('plot_opts.raceline')
        params_dict['plot_opts']['raceline'] = self.get_parameter('plot_opts.raceline').value
        self.declare_parameter('plot_opts.imported_bounds')
        params_dict['plot_opts']['imported_bounds'] = self.get_parameter('plot_opts.imported_bounds').value
        self.declare_parameter('plot_opts.raceline_curv')
        params_dict['plot_opts']['raceline_curv'] = self.get_parameter('plot_opts.raceline_curv').value
        self.declare_parameter('plot_opts.racetraj_vel')
        params_dict['plot_opts']['racetraj_vel'] = self.get_parameter('plot_opts.racetraj_vel').value
        self.declare_parameter('plot_opts.racetraj_vel_3d')
        params_dict['plot_opts']['racetraj_vel_3d'] = self.get_parameter('plot_opts.racetraj_vel_3d').value
        self.declare_parameter('plot_opts.racetraj_vel_3d_stepsize')
        params_dict['plot_opts']['racetraj_vel_3d_stepsize'] = self.get_parameter('plot_opts.racetraj_vel_3d_stepsize').value
        self.declare_parameter('plot_opts.spline_normals')
        params_dict['plot_opts']['spline_normals'] = self.get_parameter('plot_opts.spline_normals').value
        self.declare_parameter('plot_opts.mintime_plots')
        params_dict['plot_opts']['mintime_plots'] = self.get_parameter('plot_opts.mintime_plots').value
        self.declare_parameter('plot_opts.racetraj_vel_3d_simple')
        params_dict['plot_opts']['racetraj_vel_3d_simple'] = self.get_parameter('plot_opts.racetraj_vel_3d_simple').value
        return params_dict
                
    def __centerline_sub_callback(self, msg: Marker):
        self.track.set_reftrack(centerline=[
            [point.x,
             point.y,
            point.z if point.z != 0 else self.params_dict['misc']['fakeDistance'],
            point.z if point.z != 0 else self.params_dict['misc']['fakeDistance']]
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
        
        self.track.add_boundary(points=[[point.x, point.y, point.z, point.z] for point in msg.points], line=line)
        
        if(self.params_dict['misc']['debug']):
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
        
        if self.params_dict['misc']['savePointsPath']:
            self.track.points_to_file(self.params_dict['misc']['savePointsPath'])
        
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

    global_sub = GlobalPlanner()

    rclpy.spin(global_sub)

    global_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
