import numpy as np
import time
import json
import os
import trajectory_planning_helpers as tph
import copy
import matplotlib.pyplot as plt
import configparser
import pkg_resources
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "global_racetrajectory_optimization"))

import helper_funcs_glob

DEBUG = True
PLOT = True


# set import options 
imp_opts = {"flip_imp_track": False,                # flip imported track to reverse direction
            "set_new_start": False,                 # set new starting point (changes order, not coordinates)
            "new_start": np.array([0.0, -47.0]),    # [x_m, y_m]
            "min_track_width": None,                # [m] minimum enforced track width (set None to deactivate)
            "num_laps": 1}                          # number of laps to be driven (significant with powertrain-option),
                                                    # only relevant in mintime-optimization
# lap time calculation table 
lap_time_mat_opts = {"use_lap_time_mat": False,             # calculate a lap time matrix (diff. top speeds and scales)
                     "gg_scale_range": [0.3, 1.0],          # range of gg scales to be covered
                     "gg_scale_stepsize": 0.05,             # step size to be applied
                     "top_speed_range": [100.0, 150.0],     # range of top speeds to be simulated [in km/h]
                     "top_speed_stepsize": 5.0,             # step size to be applied
                     "file": "lap_time_matrix.csv"}         # file name of the lap time matrix (stored in "outputs")
# plot options
plot_opts = {"mincurv_curv_lin": False,         # plot curv. linearization (original and solution based) (mincurv only)
             "raceline": True,                  # plot optimized path
             "imported_bounds": False,          # plot imported bounds (analyze difference to interpolated bounds)
             "raceline_curv": True,             # plot curvature profile of optimized path
             "racetraj_vel": True,              # plot velocity profile
             "racetraj_vel_3d": False,          # plot 3D velocity profile above raceline
             "racetraj_vel_3d_stepsize": 1.0,   # [m] vertical lines stepsize in 3D velocity profile plot
             "spline_normals": False,           # plot spline normals to check for crossings
             "mintime_plots": False}            # plot states, controls, friction coeffs etc. (mintime only)

class Trajectory:
    def __init__(self) -> None:
        self.file_paths = {"veh_params_file": "racecar.ini"}

            
        # INITIALIZATION OF PATHS 

        self.file_paths["module"] = os.path.dirname(os.path.abspath(__file__))

        # create outputs folder(s)
        os.makedirs(self.file_paths["module"] + "/outputs", exist_ok=True)

        # assemble export paths
        self.file_paths["traj_race_export"] = os.path.join(self.file_paths["module"], "outputs", "traj_race_cl.csv")
        # file_paths["traj_ltpl_export"] = os.path.join(file_paths["module"], "outputs", "traj_ltpl_cl.csv")
        self.file_paths["lap_time_mat_export"] = os.path.join(self.file_paths["module"], "outputs", lap_time_mat_opts["file"])

  
        # IMPORT VEHICLE DEPENDENT PARAMETERS 

        # load vehicle parameter file into a "pars" dict
        parser = configparser.ConfigParser()
        self.pars = {}

        if not parser.read(os.path.join(self.file_paths["module"], "input", self.file_paths["veh_params_file"])):
            raise ValueError('Specified config file does not exist or is empty!')

        self.pars["ggv_file"] = json.loads(parser.get('GENERAL_OPTIONS', 'ggv_file'))
        self.pars["ax_max_machines_file"] = json.loads(parser.get('GENERAL_OPTIONS', 'ax_max_machines_file'))
        self.pars["stepsize_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'stepsize_opts'))
        self.pars["reg_smooth_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'reg_smooth_opts'))
        self.pars["veh_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
        self.pars["vel_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts'))
        self.pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mincurv'))
        
        self.file_paths["ggv_file"] = os.path.join(self.file_paths["module"], "input", self.pars["ggv_file"])
        self.file_paths["ax_max_machines_file"] = os.path.join(self.file_paths["module"], "input", self.pars["ax_max_machines_file"])
        
    def optimize(self, rtrack = np.array([])):
        # IMPORT TRACK AND VEHICLE DYNAMICS INFORMATION 

        # save start time
        t_start = time.perf_counter()

        # import track
        reftrack_imp = rtrack 


        ggv, ax_max_machines = tph.import_veh_dyn_info.\
        import_veh_dyn_info(ggv_import_path=self.file_paths["ggv_file"],
                            ax_max_machines_import_path=self.file_paths["ax_max_machines_file"])


        # PREPARE REFTRACK 

        reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = \
            helper_funcs_glob.src.prep_track.prep_track(reftrack_imp=reftrack_imp,
                                                        reg_smooth_opts=self.pars["reg_smooth_opts"],
                                                        stepsize_opts=self.pars["stepsize_opts"],
                                                        debug=DEBUG,
                                                        min_width=imp_opts["min_track_width"])

        # CALL OPTIMIZATION 
        pars_tmp = self.pars
        alpha_opt, reftrack_interp, normvec_normalized_interp = tph.iqp_handler.iqp_handler(
                        reftrack=reftrack_interp,
                        normvectors=normvec_normalized_interp,
                        A=a_interp,
                        kappa_bound=self.pars["veh_params"]["curvlim"],
                        w_veh=self.pars["optim_opts"]["width_opt"],
                        print_debug=DEBUG,
                        plot_debug=plot_opts["mincurv_curv_lin"],
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
                            filt_window=self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
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

        if plot_opts["racetraj_vel"]:
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

        if lap_time_mat_opts["use_lap_time_mat"]:
            # simulate lap times
            ggv_scales = np.linspace(lap_time_mat_opts['gg_scale_range'][0],
                                    lap_time_mat_opts['gg_scale_range'][1],
                                    int((lap_time_mat_opts['gg_scale_range'][1] - lap_time_mat_opts['gg_scale_range'][0])
                                        / lap_time_mat_opts['gg_scale_stepsize']) + 1)
            top_speeds = np.linspace(lap_time_mat_opts['top_speed_range'][0] / 3.6,
                                    lap_time_mat_opts['top_speed_range'][1] / 3.6,
                                    int((lap_time_mat_opts['top_speed_range'][1] - lap_time_mat_opts['top_speed_range'][0])
                                        / lap_time_mat_opts['top_speed_stepsize']) + 1)

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
                                        filt_window=self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
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

        bound1, bound2 = helper_funcs_glob.src.check_traj.\
            check_traj(reftrack=reftrack_interp,
                    reftrack_normvec_normalized=normvec_normalized_interp,
                    length_veh=self.pars["veh_params"]["length"],
                    width_veh=self.pars["veh_params"]["width"],
                    debug=DEBUG,
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
            helper_funcs_glob.src.export_traj_race.export_traj_race(file_paths=self.file_paths,
                                                                    traj_race=traj_race_cl)


        # if requested, export trajectory including map information (via normal vectors) to CSV
        if "traj_ltpl_export" in self.file_paths.keys():
            helper_funcs_glob.src.export_traj_ltpl.export_traj_ltpl(file_paths=self.file_paths,
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

        if plot_opts["imported_bounds"]:
            # try to extract four times as many points as in the interpolated version (in order to hold more details)
            n_skip = max(int(reftrack_imp.shape[0] / (bound1.shape[0] * 4)), 1)

            _, _, _, normvec_imp = tph.calc_splines.calc_splines(path=np.vstack((reftrack_imp[::n_skip, 0:2],
                                                                                reftrack_imp[0, 0:2])))

            bound1_imp = reftrack_imp[::n_skip, :2] + normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 2], 1)
            bound2_imp = reftrack_imp[::n_skip, :2] - normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 3], 1)

        # plot results
        helper_funcs_glob.src.result_plots.result_plots(plot_opts=plot_opts,
                                                        width_veh_opt=self.pars["optim_opts"]["width_opt"],
                                                        width_veh_real=self.pars["veh_params"]["width"],
                                                        refline=reftrack_interp[:, :2],
                                                        bound1_imp=bound1_imp,
                                                        bound2_imp=bound2_imp,
                                                        bound1_interp=bound1,
                                                        bound2_interp=bound2,
                                                        trajectory=trajectory_opt)

       
traj = Trajectory()