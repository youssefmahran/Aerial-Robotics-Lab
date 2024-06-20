# Installing Ardupilot SITL
## 1. Install Git
Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and
install git to be able to clone the ardupilot repo
```bash
cd ~    
sudo apt install git   
``` 

## 2. Clone Ardupilot Repo
Clone the Ardupilot repo in the home directory. To check the latest version of Arducopter visit [Arducopter](https://firmware.ardupilot.org/Copter/) and find the latest version.
```bash
cd ~    
git clone https://github.com/ArduPilot/ardupilot.git   
cd ardupilot    
git checkout Copter-<Latest Version>    #For example Copter-4.5.1
git submodule update --init --recursive
``` 

## 3. Install the Dependencies
Install the needed dependencies
```bash
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Log out and Log in again for changes to take place

Use pip (python package installer) to install mavproxy
```bash
sudo pip install future pymavlink MAVProxy
```

## 4. Running the SITL
Run the SITL (Software In The Loop) once to set params:
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
