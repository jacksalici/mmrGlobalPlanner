import numpy as np
import time
import json
import os
import trajectory_planning_helpers as tph
import copy
import configparser
import pkg_resources
import sys
from global_planner.helper import prep_track, check_traj, export_traj_race, export_traj_ltpl, interp_track



class Trajectory:
    def __init__(self, params) -> None:
        
        self.pars = params['car_config']
        self.pars['plot_opts'] = params['plot_opts']
        self.pars['lap_time_mat_opts'] = params['lap_time_mat_opts']
        self.pars['imp_opts'] = params['imp_opts']
        self.pars['misc'] = params['misc']
        self.pars["optim_opts"] = params['optimization_opt']['optim_opts_mincurv']

        self.file_paths = {"veh_params_file": "racecar.ini"}

            
        # INITIALIZATION OF PATHS 

        self.file_paths["module"] = os.path.dirname(os.path.abspath(__file__))

        # create outputs folder(s)
        os.makedirs(self.file_paths["module"] + "/outputs", exist_ok=True)

        # assemble export paths
        self.file_paths["traj_race_export"] = os.path.join(self.file_paths["module"], "outputs", "traj_race_cl.csv")
        # file_paths["traj_ltpl_export"] = os.path.join(file_paths["module"], "outputs", "traj_ltpl_cl.csv")
        self.file_paths["lap_time_mat_export"] = os.path.join(self.file_paths["module"], "outputs", self.pars['lap_time_mat_opts']["file"])

  
        # IMPORT VEHICLE DEPENDENT PARAMETERS 

        self.file_paths["ggv_file"] = os.path.join(self.file_paths["module"], "input", self.pars["ggv_file"])
        self.file_paths["ax_max_machines_file"] = os.path.join(self.file_paths["module"], "input", self.pars["ax_max_machines_file"])
        
        print(self.pars)
        
    def optimize(self, rtrack = np.array([])):
        # IMPORT TRACK AND VEHICLE DYNAMICS INFORMATION 
        if self.pars['misc']['debug']:        
            from global_planner.helper import result_plots
            import matplotlib.pyplot as plt


        # save start time
        t_start = time.perf_counter()

        # import track
        reftrack_imp = rtrack 


        ggv, ax_max_machines = tph.import_veh_dyn_info.\
        import_veh_dyn_info(ggv_import_path=self.file_paths["ggv_file"],
                            ax_max_machines_import_path=self.file_paths["ax_max_machines_file"])


        # PREPARE REFTRACK 

        reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = \
            prep_track.prep_track(reftrack_imp=reftrack_imp,
                                                        reg_smooth_opts=self.pars["reg_smooth_opts"],
                                                        stepsize_opts=self.pars["stepsize_opts"],
                                                        debug=self.pars['misc']['debug'],
                                                        min_width=self.pars['imp_opts']["min_track_width"] if self.pars['imp_opts']["min_track_width"]!=-1 else None)

        # CALL OPTIMIZATION 
        pars_tmp = self.pars
        alpha_opt, reftrack_interp, normvec_normalized_interp = tph.iqp_handler.iqp_handler(
                        reftrack=reftrack_interp,
                        normvectors=normvec_normalized_interp,
                        A=a_interp,
                        kappa_bound=self.pars["veh_params"]["curvlim"],
                        w_veh=self.pars["optim_opts"]["width_opt"],
                        print_debug=self.pars['misc']['debug'],
                        plot_debug=self.pars['plot_opts']["mincurv_curv_lin"],
                        stepsize_interp=self.pars["stepsize_opts"]["stepsize_reg"],
                        iters_min=self.pars["optim_opts"]["iqp_iters_min"],
                        curv_error_allowed=self.pars["optim_opts"]["iqp_curverror_allowed"])



        # INTERPOLATE SPLINES TO SMALL DISTANCES BETWEEN RACELINE POINTS 

        raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, t_vals_opt_interp, s_points_opt_interp,\
            spline_lengths_opt, el_lengths_opt_interp = tph.create_raceline.\
            create_raceline(refline=reftrack_interp[:, :2],
                            normvectors=normvec_normalized_interp,
                            alpha=alpha_opt,
                            stepsize_interp=self.pars["stepsize_opts"]["stepsize_interp_after_opt"])

        # CALCULATE HEADING AND CURVATURE 

        # calculate heading and curvature (analytically)
        psi_vel_opt, kappa_opt = tph.calc_head_curv_an.\
            calc_head_curv_an(coeffs_x=coeffs_x_opt,
                            coeffs_y=coeffs_y_opt,
                            ind_spls=spline_inds_opt_interp,
                            t_spls=t_vals_opt_interp)

        # CALCULATE VELOCITY AND ACCELERATION PROFILE 

        vx_profile_opt = tph.calc_vel_profile.calc_vel_profile(
                            ggv=ggv,
                            ax_max_machines=ax_max_machines,
                            v_max=self.pars["veh_params"]["v_max"],
                            kappa=kappa_opt,
                            el_lengths=el_lengths_opt_interp,
                            closed=True,
                            filt_window=self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"] if self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"] != -1 else None,
                            dyn_model_exp=self.pars["vel_calc_opts"]["dyn_model_exp"],
                            drag_coeff=self.pars["veh_params"]["dragcoeff"],
                            m_veh=self.pars["veh_params"]["mass"])

        # calculate longitudinal acceleration profile
        vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
        ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                            el_lengths=el_lengths_opt_interp,
                                                            eq_length_output=False)

        # calculate laptime
        t_profile_cl = tph.calc_t_profile.calc_t_profile(vx_profile=vx_profile_opt,
                                                        ax_profile=ax_profile_opt,
                                                        el_lengths=el_lengths_opt_interp)
        print("INFO: Estimated laptime: %.2fs" % t_profile_cl[-1])

        if self.pars['misc']['debug'] and self.pars['plot_opts']["racetraj_vel"]:
            s_points = np.cumsum(el_lengths_opt_interp[:-1])
            s_points = np.insert(s_points, 0, 0.0)

            plt.plot(s_points, vx_profile_opt)
            plt.plot(s_points, ax_profile_opt)
            plt.plot(s_points, t_profile_cl[:-1])

            plt.grid()
            plt.xlabel("distance in m")
            plt.legend(["vx in m/s", "ax in m/s2", "t in s"])

            plt.show()

        # CALCULATE LAP TIMES (AT DIFFERENT SCALES AND TOP SPEEDS) 

        if self.pars['lap_time_mat_opts']["use_lap_time_mat"]:
            # simulate lap times
            ggv_scales = np.linspace(self.pars['lap_time_mat_opts']['gg_scale_range'][0],
                                    self.pars['lap_time_mat_opts']['gg_scale_range'][1],
                                    int((self.pars['lap_time_mat_opts']['gg_scale_range'][1] - self.pars['lap_time_mat_opts']['gg_scale_range'][0])
                                        / self.pars['lap_time_mat_opts']['gg_scale_stepsize']) + 1)
            top_speeds = np.linspace(self.pars['lap_time_mat_opts']['top_speed_range'][0] / 3.6,
                                    self.pars['lap_time_mat_opts']['top_speed_range'][1] / 3.6,
                                    int((self.pars['lap_time_mat_opts']['top_speed_range'][1] - self.pars['lap_time_mat_opts']['top_speed_range'][0])
                                        / self.pars['lap_time_mat_opts']['top_speed_stepsize']) + 1)

            # setup results matrix
            lap_time_matrix = np.zeros((top_speeds.shape[0] + 1, ggv_scales.shape[0] + 1))

            # write parameters in first column and row
            lap_time_matrix[1:, 0] = top_speeds * 3.6
            lap_time_matrix[0, 1:] = ggv_scales

            for i, top_speed in enumerate(top_speeds):
                for j, ggv_scale in enumerate(ggv_scales):
                    tph.progressbar.progressbar(i*ggv_scales.shape[0] + j,
                                                top_speeds.shape[0] * ggv_scales.shape[0],
                                                prefix="Simulating laptimes ")

                    ggv_mod = np.copy(ggv)
                    ggv_mod[:, 1:] *= ggv_scale

                    vx_profile_opt = tph.calc_vel_profile.\
                        calc_vel_profile(ggv=ggv_mod,
                                        ax_max_machines=ax_max_machines,
                                        v_max=top_speed,
                                        kappa=kappa_opt,
                                        el_lengths=el_lengths_opt_interp,
                                        dyn_model_exp=self.pars["vel_calc_opts"]["dyn_model_exp"],
                                        filt_window=self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"] if self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"] != -1 else None,
                                        closed=True,
                                        drag_coeff=self.pars["veh_params"]["dragcoeff"],
                                        m_veh=self.pars["veh_params"]["mass"])

                    # calculate longitudinal acceleration profile
                    vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
                    ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                                        el_lengths=el_lengths_opt_interp,
                                                                        eq_length_output=False)

                    # calculate lap time
                    t_profile_cl = tph.calc_t_profile.calc_t_profile(vx_profile=vx_profile_opt,
                                                                    ax_profile=ax_profile_opt,
                                                                    el_lengths=el_lengths_opt_interp)

                    # store entry in lap time matrix
                    lap_time_matrix[i + 1, j + 1] = t_profile_cl[-1]

            # store lap time matrix to file
            np.savetxt(self.file_paths["lap_time_mat_export"], lap_time_matrix, delimiter=",", fmt="%.3f")

        # DATA POSTPROCESSING 

        # arrange data into one trajectory
        trajectory_opt = np.column_stack((s_points_opt_interp,
                                        raceline_interp,
                                        psi_vel_opt,
                                        kappa_opt,
                                        vx_profile_opt,
                                        ax_profile_opt))
        spline_data_opt = np.column_stack((spline_lengths_opt, coeffs_x_opt, coeffs_y_opt))

        # create a closed race trajectory array
        traj_race_cl = np.vstack((trajectory_opt, trajectory_opt[0, :]))
        traj_race_cl[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

        # print end time
        print("INFO: Runtime from import to final trajectory was %.2fs" % (time.perf_counter() - t_start))

        # CHECK TRAJECTORY 

        bound1, bound2 = check_traj.\
            check_traj(reftrack=reftrack_interp,
                    reftrack_normvec_normalized=normvec_normalized_interp,
                    length_veh=self.pars["veh_params"]["length"],
                    width_veh=self.pars["veh_params"]["width"],
                    debug=self.pars['misc']['debug'],
                    trajectory=trajectory_opt,
                    ggv=ggv,
                    ax_max_machines=ax_max_machines,
                    v_max=self.pars["veh_params"]["v_max"],
                    curvlim=self.pars["veh_params"]["curvlim"],
                    mass_veh=self.pars["veh_params"]["mass"],
                    dragcoeff=self.pars["veh_params"]["dragcoeff"])

        # EXPORT 

        # export race trajectory  to CSV
        if "traj_race_export" in self.file_paths.keys():
            export_traj_race.export_traj_race(file_paths=self.file_paths,
                                                                    traj_race=traj_race_cl)


        # if requested, export trajectory including map information (via normal vectors) to CSV
        if "traj_ltpl_export" in self.file_paths.keys():
            export_traj_ltpl.export_traj_ltpl(file_paths=self.file_paths,
                                                                    spline_lengths_opt=spline_lengths_opt,
                                                                    trajectory_opt=trajectory_opt,
                                                                    reftrack=reftrack_interp,
                                                                    normvec_normalized=normvec_normalized_interp,
                                                                    alpha_opt=alpha_opt)

        print("INFO: Finished export of trajectory:", time.strftime("%H:%M:%S"))


        # PLOT RESULTS 

        # get bound of imported map (for reference in final plot)
        bound1_imp = None
        bound2_imp = None

        if self.pars['plot_opts']["imported_bounds"]:
            # try to extract four times as many points as in the interpolated version (in order to hold more details)
            n_skip = max(int(reftrack_imp.shape[0] / (bound1.shape[0] * 4)), 1)

            _, _, _, normvec_imp = tph.calc_splines.calc_splines(path=np.vstack((reftrack_imp[::n_skip, 0:2],
                                                                                reftrack_imp[0, 0:2])))

            bound1_imp = reftrack_imp[::n_skip, :2] + normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 2], 1)
            bound2_imp = reftrack_imp[::n_skip, :2] - normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 3], 1)

        # plot results
        if self.pars['misc']['debug']:
            result_plots.result_plots(plot_opts=self.pars['plot_opts'],
                                                        width_veh_opt=self.pars["optim_opts"]["width_opt"],
                                                        width_veh_real=self.pars["veh_params"]["width"],
                                                        refline=reftrack_interp[:, :2],
                                                        bound1_imp=bound1_imp,
                                                        bound2_imp=bound2_imp,
                                                        bound1_interp=bound1,
                                                        bound2_interp=bound2,
                                                        trajectory=trajectory_opt)
        
        self._trajectory_opt = {
            "raceline": raceline_interp,
            "speed": vx_profile_opt,
            "acceleration": ax_profile_opt,
            "heading": psi_vel_opt
        }
        return "Optimization finished"

    def get_trajectory_opt(self):
        assert self._trajectory_opt, "Output trajectory called before initialization."
        return self._trajectory_opt
        
        
#for debug
if __name__ == "__main__":
    import argparse, pathlib, time, yaml, scipy
    
    print("-------------\nGlobal Planner\n-------------")
    print(f"Numpy Version: {np.__version__}\nScipy Version: {scipy.__version__}\n-------------")
    
    parser = argparse.ArgumentParser(description='Global Planner tester tool')
    
    parser.add_argument('reftrack_path', type=pathlib.Path,
                    help='The path of the CSV with the (n,4)-sized array.')
    parser.add_argument('--x_mul', type=float, help='Multiplicator to rescale the track width.', default= 1.0)
    parser.add_argument('--y_mul', type=float, help='Multiplicator to rescale the track dimensions.', default= 1.0 )
    
    parser.add_argument('--save_path', type=pathlib.Path, help='Output saving folder')
    parser.add_argument('--times', type=int, help='Times to repeat the optimizations (for reliability and stability reasons)', default= 1)
    
    parser.add_argument('--plot', help='Plot the result.', action="store_true")
    

    args = parser.parse_args()
    files_path = []
    
    yaml_file_path = "config/params.yaml"

    # Load parameters from the YAML file
    params = yaml.safe_load(open(yaml_file_path, 'r'))
    
    traj = Trajectory(params['/global_planner']['ros__parameters'])
    for i in range(args.times):
        points = np.loadtxt(args.reftrack_path, delimiter=',')
        points[:,0:2]=points[:,0:2]*args.y_mul
        points[:,2:4]=points[:,2:4]*args.y_mul
        print(f"Optimizing an array of point {points.shape}")
        traj.optimize(points)
        
        if args.save_path:
            opt = traj.get_trajectory_opt()
            stack = np.column_stack((opt["raceline"], opt["speed"]))
            my_time = time.localtime()
            files_path.append(pathlib.Path.joinpath(args.save_path, time.strftime("GP_%Y%m%d_%H%M%S.csv", my_time)))
            np.savetxt(files_path[-1], stack, delimiter=",")
         
       
    def plot_double_graph(files):
        import csv

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Plot all third columns in 2D graph
        for file in files:
            with open(file, 'r') as csvfile:
                reader = csv.reader(csvfile)
                data = list(reader)
                third_column = [float(row[2]) for row in data]  # Assuming third column index is 2
                ax1.plot(third_column, label=os.path.basename(file).split('.')[0])
        ax1.set_title('Speed profile')
        ax1.legend()

        # Plot all first and second columns
        for file in files:
            with open(file, 'r') as csvfile:
                reader = csv.reader(csvfile)
                data = list(reader)
                first_column = [float(row[0]) for row in data]  # Assuming first column index is 0
                second_column = [float(row[1]) for row in data]  # Assuming second column index is 1
                ax2.plot(first_column, second_column, label=os.path.basename(file).split('.')[0])
        ax2.set_title('New Trajectory')
        ax2.legend()

        plt.tight_layout()
        plt.show()
        
    if args.plot and args.save_path:
        plot_double_graph(files=files_path)
