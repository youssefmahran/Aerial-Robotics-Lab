# Installing QGroundControl
## 1. Configuring Ubuntu
Before installing QGroundControl you should remove the modem manager and grant yourself permissions to access the serial port. You also need to install GStreamer in order to support video streaming.

Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and
run the following commands
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y
``` 
Logout from ubuntu and login again to enable the changes

Download [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) from the QGroundControl website

Copy the installed file to the home directory then run the following commands in the terminal
```
cd ~    #Go to home Directory
chmod +x ./QGroundControl.AppImage  #Make the file executable
./QGroundControl.AppImage  (or double click)    #Run QgroundControl
``` 

## 2. Run SITL and connect with QGroundControl
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

## 3. Launch Script
For ease of use and shorter launch command create a script to launch QGC.

Open gedit to edit `qgc.sh`
```
cd ~
gedit ~/qgc.sh
```

In gedit add the following lines
```
lipsum
```
Save the file and close

In the terminal run make the script executable
```
chmod +x ./qgc.sh
```

Launch QGC from the script by running the following command
```
./qgc.sh
```