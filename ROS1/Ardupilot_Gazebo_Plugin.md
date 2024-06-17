# Installing Ardupilot Gazebo
## 1. Make Sure Gazebo is Installed
Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and run the following command
```
sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins
```

## 2. Install Gazebo Plugin for ArduPilot Master
Clone the following repo
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
```

## 3. Build and Install the Plugin
```
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

## 4. Edit the bash File
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

## 5. Run the Simulator
In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

In the SITL terminal (Terminal 2) wait for the `EK3 is using GPS1` message then run
```
mode GUIDED
arm throttle
takeoff 3
```

You can see that the drone is taking off in Gazebo

## 6. Launch Script
For ease of use and shorter launch command create a script to launch Gazebo.

Open gedit to edit `gaz.sh`
```
cd ~
gedit ~/gaz.sh
```

In gedit add the following lines
```
#!/bin/bash
pkill gzserver
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```
Save the file and close

In the terminal make the script executable
```
chmod +x ./gaz.sh
```

Create a script to launch ArduPilot SITL.

Open gedit to edit `sitl.sh`
```
cd ~
gedit ~/sitl.sh
```

In gedit add the following lines
```
#!/bin/bash
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
Save the file and close

In the terminal make the script executable
```
chmod +x ./sitl.sh
```

In one terminal launch gazebo
```
./gaz.sh
```

In another terminal launch the ArduPilot SITL
```
./sitl.sh
```