# Installing ROS1 Noetic
## 1. Configure Your Ubuntu Repositories
Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and configure Ubuntu repositories to allow "restricted," "universe," and "multiverse"
```bash
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse
```

Update package list
```bash
sudo apt update
```

## 2. Setup sources.list
Setup your computer to accept software from packages.ros.org.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## 3. Setup Keys
Setup your keys using
```bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

## 4. Installation
Install ros using
```bash
sudo apt install ros-noetic-desktop-full
```

## 5. Environment Setup
Source the script in bash
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
 
## 6. Installing Dependencies
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately.

To install and intialize the needed dependencies
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

## 7. Setting Up Catkin Workspace
Install catkin
```bash
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```

Initialize the catkin workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

Edit `~/.bashrc`
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo $ROS_PACKAGE_PATH /home/<youruser>/catkin_ws/src:/opt/ros/noetic/share
```

Source `~/.bashrc`
```bash
source ~/.bashrc
```

## 8. Installing Catkin Dependencies
Install `mavros` and `mavlink`
```bash
cd ~/catkin_ws
wstool init ~/catkin_ws/src
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
catkin_make
```

Install geographiclib dependancy
```bash
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

Build the workspace
```bash
cd ~/catkin_ws
catkin_make
```
