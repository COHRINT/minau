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
If you get an error saying the commit is not available or does not exist, try pulling the latest

## Configuration
Each bluerov instance has its own config file located in config/
Currently only waypoint navigation is supported, this can be configured via the action# param
```
point: [2,2,2]
```
means the sub will navigate to point (2,2,2) in NED frame
Once arriving it will go onto the next action# -> action(#+1)
If an action contains a repeat: #. This will loop the action back to a previous action#.

## Launching
To run a point simulator
```
roslaunch cohrint_minau bluerovs.launch
```
( press play in Gazebo )