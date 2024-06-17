# Adding Aerial Robotics Lab Environment to Gazebo
## 1. Download Needed File
Download the [`Lab.zip`](Lab.zip) File

Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and run this command
```
sudo apt install unzip
unzip Lab.zip -d ~/Downloads
```

## 2. Move the Files
Run the following Commands
```
mkdir ~/.gazebo/models/GLab
mkdir ~/.gazebo/models/GLab/meshes
cd ~/Downloads
mv model.config model.sdf ~/.gazebo/models/GLab
mv model.dae ~/.gazebo/models/GLab/meshes
mv Lab.world ~/ardupilot_gazebo/worlds
```

## 3. Run the Simulation
Edit the `gaz.sh` script
```
cd ~
gedit gaz.sh
```

Replace the `iris_arducopter_runway.world` with `Lab.world` in the 3rd line
```
gazebo --verbose ~/ardupilot_gazebo/worlds/Lab.world
```

In one terminal launch gazebo
```
./gaz.sh
```

In another terminal launch the ArduPilot SITL
```
./sitl.sh
```

In the SITL terminal (Terminal 2) wait for the `EK3 is using GPS1` message then run
```
mode GUIDED
arm throttle
takeoff 1
```

You can see that the drone is taking off inside the simulated Aerial Robotics Lab environment