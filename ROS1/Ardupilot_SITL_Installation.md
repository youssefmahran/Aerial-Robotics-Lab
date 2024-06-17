# Installing QGroundControl
Before installing QGroundControl you should remove the modem manager and grant yourself permissions to access the serial port. You also need to install GStreamer in order to support video streaming.

Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and
run te=he following commands
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