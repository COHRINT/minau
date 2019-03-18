# minau


## Files
```
scripts/point_sim_controler.py 
```
This file takes in an Odometry waypoint goal and simulates the motion of the point as it travels to the odom waypoint goal. Because it abstracts thruster controls, it takes the place of a controler and because it runs the motion simulation, it is also a simulator. It is configurable via params/points.yaml

```
scripts/point_planner.py
```
This file publishes Odometry waypoint goals. It is configurable via params/points.yaml 
