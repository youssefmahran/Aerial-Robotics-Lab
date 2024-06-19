# Adding Aerial Robotics Lab Environment to Gazebo
## 1. Download Needed File
Download the [`Lab.zip`](Lab.zip) File

Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and run this command
```bash
cd ~
sudo apt install unzip
unzip Lab.zip -d ~/Downloads
```

## 2. Move the Files
Run the following Commands
```bash
mkdir ~/.gazebo/models/GLab
mkdir ~/.gazebo/models/GLab/meshes
cd ~/Downloads
mv model.config model.sdf ~/.gazebo/models/GLab
mv model.dae ~/.gazebo/models/GLab/meshes
mv Lab.world ~/ardupilot_gazebo/worlds
```

## 3. Run the Simulation
Edit the `gaz.sh` script
```bash
cd ~
gedit gaz.sh
```

Replace the `iris_arducopter_runway.world` with `Lab.world` in the 3rd line
```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/Lab.world
```
Save `gaz.sh` and close it

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

Wait for the `EK3 is using GPS1` message to appear in the SITL terminal or its console then run
```
mode GUIDED
arm throttle
takeoff 1
```

You can see that the drone is taking off inside the simulated Aerial Robotics Lab environment