# Using MAVROS
## 1. Installing MAVROS and MAVLink
Install `mavros`
```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
```

Install geographiclib dependancy
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## 2. Creating apm.launch File
Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and create a launch folder
```bash
cd ~
mkdir ~/launch
```

Edit `apm.launch` file
```bash
cd ~/launch
gedit apm.launch
```

In gedit add the following
```
<launch>
	<!-- vim: set ft=xml noet : -->
	<!-- example launch script for ArduPilot based FCU's -->

	<arg name="fcu_url" default="udp://127.0.0.1:14551@14555" />
	<arg name="gcs_url" default="" />
	<arg name="tgt_system" default="1" />
	<arg name="tgt_component" default="1" />
	<arg name="log_output" default="screen" />
	<arg name="fcu_protocol" default="v2.0" />
	<arg name="respawn_mavros" default="false" />

	<include file="$(find mavros)/launch/node.launch">
		<arg name="pluginlists_yaml" value="$(find mavros)/launch/apm_pluginlists.yaml" />
		<arg name="config_yaml" value="$(find mavros)/launch/apm_config.yaml" />

		<arg name="fcu_url" value="$(arg fcu_url)" />
		<arg name="gcs_url" value="$(arg gcs_url)" />
		<arg name="tgt_system" value="$(arg tgt_system)" />
		<arg name="tgt_component" value="$(arg tgt_component)" />
		<arg name="log_output" value="$(arg log_output)" />
		<arg name="fcu_protocol" value="$(arg fcu_protocol)" />
		<arg name="respawn_mavros" value="$(arg respawn_mavros)" />
	</include>
</launch>
```
Save and close apm.launch

## 2. Running MAVROS
In one terminal launch gazebo
```bash
./gaz.sh
```
OR using
```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/Lab.world
```

In another terminal launch the ArduPilot SITL
```bash
./sitl.sh
```
OR using
```bash
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

Wait for the SITL to initialize then in a third terminal launch apm.launch
```bash
cd ~/launch
roslaunch apm.launch
```

Now MAVROS is publishing the topics needed for us to control the quadcopter autonomously from C++ or Python scripts

To see the available topics, in a fourth terminal list the topics
```bash
rostopic list
```

You can see all the topics that are being published through MAVROS

To see the messages that are being published to a certain topic (ex: local position), echo the topic
```bash
rostopic echo /mavros/local_position/pose
```

You can see the position and orientation of the drone being published
```
---
header: 
  seq: 3
  stamp: 
    secs: 1718799678
    nsecs: 160962544
  frame_id: "map"
pose: 
  position: 
    x: 0.010833407752215862
    y: -0.011682561598718166
    z: -0.027400298044085503
  orientation: 
    x: -0.0009441190514979661
    y: 3.110971119756951e-05
    z: -0.696722086689393
    w: -0.7173405339102066
---
```

In the SITL terminal takeoff the drone and notice how the MAVROS position message changes as the altitude changes
```
mode GUIDED
arm throttle
takeoff 1
```
## 3. Launch Script
For ease of use and shorter launch command create a script to launch MAVROS.

Open gedit to edit `mav.sh`
```bash
cd ~
gedit ~/mav.sh
```

In gedit add the following lines
```
#!/bin/bash
cd ~/launch && roslaunch apm.launch
```
Save the file and close

In the terminal make the script executable
```bash
chmod +x ./mav.sh
```

Launch MAVROS from the script by running the following command
```bash
./mav.sh
```

## 4. Launching All Needed Terminals From a Single Command

Create a script that launches all needed terminals for autonomous flight (NOTE you need to create all needed scripts found in [Ardupilot Gazebo Plugin](Ardupilot_Gazebo_Plugin.md) and [QGroundControl Installation](QGroundControl_Installation.md) and the above MAVROS script)

Open gedit to edit `sim.sh`
```bash
cd ~
gedit ~/sim.sh
```

In gedit add the following lines
```
#!/bin/bash
gnome-terminal --tab --title="Gazebo" --command="./gaz.sh"

gnome-terminal --tab --title="SITL" --command="./sitl.sh"

gnome-terminal --tab --title="QGroundControl" --command="./qgc.sh"
sleep 20

gnome-terminal --tab --title="MAVROS" --command="./mav.sh"
```
Save the file and close

In the terminal make the script executable
```bash
chmod +x ./sim.sh
```

Launch Gazebo, SITL, QGroundCrontol and MAVROS by running a single command (Note that MAVROS terminal launches after a 20 seconds delay as MAVROS requires the SITL to be completely initialized)
```bash
./sim.sh
```
