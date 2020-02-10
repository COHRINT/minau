# minau

#### Fake GPS Procedure
- Source the appropriate ROS workspace
- Launch the 'minimal_mavros.launch' file with **your** IP address as an argument:

```python
roslaunch minau minimal_mavros.launch gcs_ip:=192.168.8.213 
```
Firstly this will run the *seatrac_fake_gps_gen.py* node which publishes a PoseStamped message to the */mavros/fake_gps/mocap/pose* topic. The fake_gps plugin converts the pose into a GPS message and sends it as a MAVLINK message to the autopilot.

This will launch the *mavros.launch* file. This file looks for and loads two configuration files:
**mavros_apm_pluginlist.yaml**: This pluginlist includes a blacklist of plugins that mavros should ignore and a whitelist of plugins that mavros should load and initialize. Make sure to put the *fake_gps* plugin under the whitelist.
		
**mavros_apm_config.yaml**: This config file includes config parameters for all the plugins used. The *fake_gps* section has the following important parameters:
> Data source: (ensure *use_mocap* is set to true and the rest to false)

> Geo Origin: This initializes the sub to a specified lat, long, and alt

- Run QGroundControl Mission Planner (ensure this step is done last)
	- Ensure the **GPS_TYPE** ardusub parameter is set to **MAV**

#### Running Fake GPS with Ardusub Simulation
- Follow the instructions in this link: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
