# Plotting Flight Data
## 1. Run a SITL with a Script
First run gazebo, SITL and MAVROS either manually or using the simulation script we worte in [MAVROS](MAVROS.md)
```bash
cd ~
./sim.sh
```

Wait until MAVROS is launched and initialized.

Once MAVROS is intitialized make sure the rostopics are being published
```bash
rostopic list
```

Record a MAVROS topic using the following command in a new terminal
```bash
rosbag record -O <output filename>.bag </mavros/topic1> </mavros/topic2>
```

For this tutorial we want to record the current position of the drone and the setpoints published to the drone to make a desired vs actual trajectory plot

In our case we will record `/mavros/local_position/pose` (as actual trajectory) and `mavros/setpoint_raw/local` (as desired trajectory)
```bash
cd ~
rosbag record -O position.bag /mavros/local_position/pose mavros/setpoint_raw/local
```

After the `EK3 is using GPS` message appears in the SITL terminal or its console run a script. In this tutorial we will use the python square script
```bash
python3 square.py
```
Wait for the drone to land and then in the `rosbag` terminal press `CTRL`+`C` to end the recording process


## 2. Converting rosbag to CSV
The rosbag is located in the folder that was open in the recording terminal. In our case it is the home folder.

Download the following [rosbag_to_csv](https://github.com/AtsushiSakai/rosbag_to_csv) tool

Run the tool

```bash
rosrun rosbag_to_csv rosbag_to_csv.py
```

Select your rosbag file and output each topic to a seperate CSV file

Plot the produced CSV files using MATLAB or any desired plotting platforms
