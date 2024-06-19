
# Installing Ubuntu 22.04 as Dual Boot
## 1. Create a Bootable USB
- Download [RUFUS](https://rufus.ie/en/) to flash the USB

- Download [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) iso image

- Open RUFUS and select your USB as well as the iso image and press start

<img src="Photos/pic1.png" alt="Example Image" width="232" height="290">

- Choose the following options in the pop up menu and press ok

<img src="Photos/pic2.png" alt="Example Image" width="262" height="145">

- Wait for the operation to end then close RUFUS

## 2. Creating an Empty Partition
- Open Disk Management in Windows and locate the partition you want to shrink

<img src="Photos/pic3.png" alt="Example Image" width="378" height="298">

- Write click on the partition and select Shrink Volume

<img src="Photos/pic4.png" alt="Example Image" width="378" height="328">

- Choose the amount of free space you want and press shrink (Recommended at least 100GB which is 102400MB)

<img src="Photos/pic5.png" alt="Example Image" width="226" height="153">

- You can see the unallocated space for Ubuntu

<img src="Photos/pic6.png" alt="Example Image" width="377" height="298">

## 2. Installing Ubuntu
- Restart the computer and enter the boot menu by repeatedly pressing `F12` on Lenovo and DELL Laptops and `F9` on HP Laptops

- Select your USB device as a boot device

- When Ubunutu loads choose `Ubuntu` Option

- Wait for Ubuntu to open then press Install Ubuntu

<img src="Photos/1.jpg" alt="Example Image" width="576" height="432">

- Choose English in both options and press continue

<img src="Photos/2.jpg" alt="Example Image" width="576" height="432">

- Connect to a WiFi network and press continue

<img src="Photos/3.jpg" alt="Example Image" width="576" height="432">

- Choose Normal Installation and uncheck any other option and press continue

<img src="Photos/4.jpg" alt="Example Image" width="576" height="432">

- Choose Something else and press continue

<img src="Photos/5.jpg" alt="Example Image" width="576" height="432">

- Locate the free space we created in Windows (Look for the size) and press the `+` button to create different partitions

<img src="Photos/6.jpg" alt="Example Image" width="576" height="432">

- Create a swap area as a logical partition at the beginning of this space as shown. The swap size is at least half the RAM size and at most double the RAM size. So for an 8GB RAM the minimum is 4GB and maximum is 16GB 

<img src="Photos/8.jpg" alt="Example Image" width="576" height="432">

- Create a root folder as a logical partition at the beginning of this space and mount point `/` as shown. The recommended root folder size is 30GB (30720MB)

<img src="Photos/7.jpg" alt="Example Image" width="576" height="432">

- Allocate the remaining space for the home folder as a logical partition at the beginning of this space and mount point `/home` as shown.

<img src="Photos/9.jpg" alt="Example Image" width="576" height="432">

- You should have the partitions as shown. Choose your hard drive as the boot loader installation device and press install now

<img src="Photos/10.jpg" alt="Example Image" width="576" height="432">

- Choose your country and press continue

<img src="Photos/11.jpg" alt="Example Image" width="576" height="432">

- Finally fill out your info and press continue

<img src="Photos/12.jpg" alt="Example Image" width="576" height="432">

- When the installation finishes press restart

## 3. After Installation
Boot into Ubuntu and open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and execute the following commands. You will be asked to enter your password
```bash
sudo apt update && sudo apt upgrade -y
```

In order to play media files like MP#, MPEG4, AVI etc, you’ll need to install media codecs. Ubuntu has them in their repository but doesn’t install it by default because of copyright issues in various countries
```bash
sudo apt install ubuntu-restricted-extras
```