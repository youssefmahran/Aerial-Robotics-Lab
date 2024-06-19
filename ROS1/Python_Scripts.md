# Running Python Scripts
## 1. Download the Python Scripts
Download the python scripts found in [Python scripts](Example_Codes/Python)

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

In another terminal go to the directory where you downloaded your scripts
```bash
cd /path/to/python/scripts
```

After the `EK3 is using GPS` message appears run the C++ script
```bash
python3 circle.py
```
OR
```bash
python3 helix.py
```
OR
```bash
python3 square.py
```
You will see the drone being armed, taking off and flying in a circle all autonmously


## 2. Writing Your Own Scripts
In the [Python Scripts](Example_Codes/Python) folder you will find 3 scripts fully written with line-by-line explanation for each line

Use these scripts as a guide to write your own script

After writing your script run it using the following command

```bash
python3 <file name>.py
```

Replace `<filename>` with the name of your python file