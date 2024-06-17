# Installing Ardupilot SITL
## Install Git
Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and
install git to be able to clone the ardupilot repo
```
cd ~    #Go to home Directory
sudo apt install git    #Install git 
``` 

## Clone Ardupilot Repo
Clone the Ardupilot repo in the home directory. To check the latest version of Arducopter visit [Arducopter](https://firmware.ardupilot.org/Copter/) and find the latest version.
```
cd ~    #Go to home Directory
git clone https://github.com/ArduPilot/ardupilot.git    #Clone the repo
cd ardupilot    #Go to ardupilot directory
git checkout Copter-<Latest Version>    #For example Copter-4.5.1
git submodule update --init --recursive
``` 

## Install the Dependencies
Install the needed dependencies
```
sudo apt install python3-matplotlib python3-serial python-wxgtk3.0 python-wxtools python3-lxml python3-scipy python3-opencv ccache gawk python3-pip python3-pexpect python-is-python3
```


Use pip (python package installer) to install mavproxy
```
sudo pip install future pymavlink MAVProxy
```

## Edit bashrc
Open `~/.bashrc` to edit it
```
gedit ~/.bashrc
```


Add the following lines at the end of the `~/.bashrc` file
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```
Save and close the editor


Source `~/.bashrc` by running this in the terminal
```
.~/.bashrc
```

## Running the SITL
Run the SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```