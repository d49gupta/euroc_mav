# Gazebo sim: stereo + 3D lidar + IMU + ground truth

This sets up a Gazebo (gz-sim) simulation that publishes the sensor and
ground-truth topics listed below. It does not include any lidar/vision
processing, fusion, or estimation code -- that's a separate step.

Tested against Gazebo Harmonic / Jetty conventions (the `gz sim` line, not
Gazebo Classic). Requires `ros_gz_sim` and `ros_gz_bridge` for your ROS 2
distro (e.g. `sudo apt install ros-$ROS_DISTRO-ros-gz`).

## Layout

```
sim/
  worlds/euroc_mav_sim.sdf   world + "mav_rig" model (sensors + plugins)
  config/bridge.yaml         ros_gz_bridge topic map
  launch/sim.launch.py       starts gz sim + the bridge together
```

## Build & run

From your colcon workspace root (this package is `sensor_pkg`):

```
colcon build --packages-select sensor_pkg
source install/setup.bash
ros2 launch sensor_pkg sim.launch.py
```

A Gazebo window should come up showing a small room with a black "mav_rig"
box floating in the middle. Check the bridge came up cleanly:

```
ros2 topic list
```

You should see: `/imu`, `/lidar/points`, `/lidar/scan`,
`/stereo/left/image_raw` (+`/camera_info`), `/stereo/right/image_raw`
(+`/camera_info`), `/ground_truth/odom`, `/clock`.

## Driving the rig

`mav_rig` has no flight dynamics -- it's moved directly via the
`VelocityControl` plugin, which listens on `/cmd_vel`:

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"
```

or drive it interactively with `teleop_twist_keyboard`.

## Recording data

```
ros2 bag record -a --use-sim-time
```

(or list the specific topics above instead of `-a`). `/ground_truth/odom`
is read straight from the physics state each step -- zero measurement
noise, not an estimate -- so it's directly comparable to what
`compare_trajectories.py` expects as ground truth, just as a live/bag
`nav_msgs/Odometry` topic instead of a EuRoC-style CSV. You'll need a small
script to either convert the bag into that CSV format or point the
comparison code at the bag directly; that conversion isn't included here.

## Known limitations / things you'll likely want to change

- **Scene texture**: the room and props are flat-colored primitives, just
  enough for a feature detector to find edges/corners. This is much sparser
  than a real environment or the actual EuRoC rooms -- if your VO front end
  struggles to track features, add real image textures to the walls/props.
- **Lidar**: configured as a 16-line 3D scan (360 deg horizontal x ~30 deg
  vertical FOV, 30 m range) to loosely resemble a small 3D lidar. Tune
  `<lidar>` in the SDF to match whatever you're trying to emulate.
- **IMU noise**: left at the sensor's defaults, which will not match any
  real IMU's calibrated noise density/random walk. If you're comparing
  against EuRoC-tuned filter parameters, you'll want to set matching
  Gaussian noise values on the `imu` sensor block.
- **No flight dynamics**: `mav_rig` is a kinematic box driven by
  `VelocityControl`, not a physically simulated multirotor.
