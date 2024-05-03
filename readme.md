# MRR Global Planner

UNIMORE driverless Formula Student ROS package for the global planning of the track.



```sh

#build (from the ROS workspace folder):
colcon build --packages-select global_planner --continue-on-error --symlink-install

#run
ros2 launch global_planner global_planner.launch.py

```

### Race Trajectory
The output contains the global race trajectory. The array is of size [`no_points` x 7] where `no_points` depends on stepsize and track length. The seven columns are structured as follows:

* `s_m`: float32, meter. Curvi-linear distance along the raceline.
* `x_m`: float32, meter. X-coordinate of raceline point.
* `y_m`: float32, meter. Y-coordinate of raceline point.
* `psi_rad`: float32, rad. Heading of raceline in current point from -pi to +pi rad. Zero is north (along y-axis).
* `kappa_radpm`: float32, rad/meter. Curvature of raceline in current point.
* `vx_mps`: float32, meter/second. Target velocity in current point.
* `ax_mps2`: float32, meter/second^2. Target acceleration in current point. We assume this acceleration to be constant from current point until next point.


# References
> **Minimum Curvature Trajectory Planning**\
Heilmeier, Wischnewski, Hermansdorfer, Betz, Lienkamp, Lohmann\
Minimum Curvature Trajectory Planning and Control for an Autonomous Racecar