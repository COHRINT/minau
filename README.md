# minau

## Configuration
Edit appropriate values in params/points.yaml

## Launching
To run a point simulator
```
roslaunch cohrint_minau points.launch
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