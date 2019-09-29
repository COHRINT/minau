# minau

Interface repo between et-ddf and uuv simulator

# Setup Correct Versions of other repos
Clone onboard, bluerov2, wamv, uuv_simulator from the Orbit Logic locally hosted repo

Development for more complicated scenarios has been done using the following repo versions
```
cd onboard
git checkout 05cc571f5bc1ad34f1171e2e7ff019f02bd09294

cd bluerov2
git checkout e205ce6840162ab24decec96182315a37ff0968f

cd uuv_simulator
git checkout aadee6a6896d4225e3a42326065e291500cdf6ab

cd wamv
git checkout 4e9f58233f792b9fd23ebfd7fc04040c613632d3
```

## Configuration
Edit appropriate values in config/gazebo_config.yaml

## Launching
To run a point simulator
```
roslaunch cohrint_minau bluerovs.launch
```
( press play in Gazebo )
```
roslaunch cohrint_minau planner.launch
```

## Files
```
params/points.yaml
```
This file is the main configuration file for the point simulator.
```
scripts/point_sim.py 
```
This file tracks points in space. An object's velcoity is modifiable via Twist msgs on the 'robot-name/new\_twist' topic. It publishes Odometry msgs to the 'robot-name/pose\_gt' topic. It is configurable under the sim tag in params/points.yaml.
```
scripts/point_planner.py
```
This file publishes Twist msgs to the 'robot-name/new\_twist' topic. It is configurable via the 'planners' tag and under each robot's configuration tags in the params/points.yaml.
```
scripts/publish_sensors.py
```
This file simulates measurement readings by publishing Measurement.msg and depth measurements using ground truth data from a simulator. Listens to /robot-name/pose_gt