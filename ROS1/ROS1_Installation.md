# Installing ROS1 Noetic
## Configure Your Ubuntu Repositories
Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and configure Ubuntu repositories to allow "restricted," "universe," and "multiverse"
```
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse
```

Update package list
```
sudo apt update
```

## Setup sources.list
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Setup Keys
Setup your keys using
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

## Installation
Install ros using
```
sudo apt install ros-noetic-desktop-full
```

## Environment Setup
Source the script in bash
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
 
## Installing Dependencies
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately.

To install and intialize the needed dependencies
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```
