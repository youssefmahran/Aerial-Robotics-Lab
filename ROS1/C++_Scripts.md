# Using MAVROS
## 1. Creating a catkin Package
Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and go to the catkin workspace created in [ROS Installation](ROS1_Installation.md)
```bash
cd ~/catkin_ws
```
Use the `catkin_create_pkg` script to create a new package called `code` which depends on `std_msgs`, `roscpp`, and `rospy`: 
```bash
catkin_create_pkg code std_msgs rospy roscpp
```

This will create a `code` folder which contains a `package.xml` and a `CMakeLists.txt`, which have been partially filled out with the information you gave `catkin_create_pkg`. 

## 2. Building Workspace
Now you need to build the packages in the catkin workspace: 
```bash
cd ~/catkin_ws
catkin_make
```

To add the workspace to your ROS environment you need to source the generated setup file
```bash
. ~/catkin_ws/devel/setup.bas
```

## 3. Adding C++ Scripts
In order to run any C++ script it must be added to `~/catkin_ws/src/code/src` folder

For the purpose of this tutorial, download one of the [C++ scripts](Example_Codes/C++)

Copy the downloaded script to `~/catkin_ws/src/code/src` folder either manually or by running the following command in the terminal
```bash
cd ~/Downloads
mv circle.cpp ~/catkin_ws/src/code/src
```

## 4. Building the Workspace
In order to run any C++ script inside the `~/catkin_ws/src/code/src` folder, it must be marked as executable to the catkin builder

This is achieved through editing the `CMakeLists.txt` file found inside the `~/catkin_ws/src/code/` folder

Navigate manually to `~/catkin_ws/src/code/` and open `CMakeLists.txt` or open it for editing through the following
```bash
gedit ~/catkin_ws/src/code/CMakeLists.txt
```

Add the following two lines at the end of the `CMakeLists.txt` file
```
add_executable(circle src/circle.cpp)
target_link_libraries(circle ${catkin_LIBRARIES})
```

Do the same for the `helix.cpp` and `square.cpp`
```
add_executable(helix src/helix.cpp)
target_link_libraries(helix ${catkin_LIBRARIES})

add_executable(square src/square.cpp)
target_link_libraries(square ${catkin_LIBRARIES})
```

Repeat this step for every C++ script you add to the `~/catkin_ws/src/code/src` folder. However, the two lines added should be edited as follows
```
add_executable(<executable name> src/<file name>.cpp)
target_link_libraries(<executable name> ${catkin_LIBRARIES})
```

Finally build the workspace
```bash
cd ~/catkin_ws
catkin_make
```

## 5. Running the Script
In order to run the C++ script, first run gazebo, SITL and MAVROS either manually or using the simulation script we worte in [MAVROS](MAVROS.md)
```bash
cd ~
./sim.sh
```

Wait until MAVROS is launched and initialized.

Once MAVROS is intitialized make sure the rostopics are being published
```bash
rostopic list
```

After the `EK3 is using GPS` message appears run the C++ script
```bash
rosrun code circle
```
OR
```bash
rosrun code helix
```
OR
```bash
rosrun code square
```

You will see the drone being armed, taking off and flying in a circle all autonmously

The format for running the script
```bash
rosrun <package name> <executable name>
```
The `<package name>` is `code` as we created in the first step while the `<executable name>` is the name of the executable you set in the previous step.

## 6. Writing Your Own Scripts
In the [C++ scripts](Example_Codes/C++) folder you will find 3 scripts fully written with line-by-line explanation for each line

Use these scripts as a guide to write your own script

After writing your script move it to the `~/catkin_ws/src/code/src` folder

Then add the following two lines to the end of the `CMakeLists.txt` found in`~/catkin_ws/src/code/`
```
add_executable(<executable name> src/<file name>.cpp)
target_link_libraries(<executable name> ${catkin_LIBRARIES})
```

Build the workspace (NOTE you have to `catkin_make` every time you do any change to your C++ scripts. If you don't `catkin_make` the changes you do in your script won't appear)
```bash
cd ~/catkin_ws
catkin_make
```

Finally, run your script
```bash
rosrun <package name> <executable name>
```