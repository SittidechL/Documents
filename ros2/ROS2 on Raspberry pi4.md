## Etcher
* [Etcher download](https://www.balena.io/etcher/)
* [How to install Etcher "image writer"](https://linuxhint.com/etcher-image-writer-ubuntu-burn-images/)
## Ubuntu on PI4
* [Unbutu20.04 Download](https://ubuntu.com/download/raspberry-pi)
* [How to Install Ubuntu on PI4](https://www.youtube.com/watch?v=1-IiLA8chCA)

```
ping google.com
sudo apt install htpdate
date
timedatectl list-timezones
timedatectl set-timezone Etc/GMT+7
sudo apt update && sudo apt upgrade
sudo apt autoremove
sudo reboot
```
---
## Install ROS2 on Raspberry PI4
[05: Setting up ROS 2 on a Raspberry PI 4](https://www.youtube.com/watch?v=eCknRpMj9uc)<br>
1. Raspberry Pi imager on Ubuntu
```
cd Downloads
sudo dpkg -i imager_1.7.1_amd64.deb
```
- after finished then check on ***Show Application***
2. Access in Rapberry pi4 by SSH command line
```
ssh ubuntu@192/168.1.107
username: ubuntu
password: ubuntu
oldpassword:ubuntu
newpassword:********
```
- reboot by it self and one more time to try again SSH
```
ssh ubuntu@192/168.1.107
username: ubuntu
password: *******
```
- go to ***ubuntu@ubuntu:~$***
```
sudo apt upgrade
sudo apt update
```

## [Installing ROS2 (Foxy) via Debian Packages](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
- Setup Sources
```
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
- Install ROS 2 packages
```
sudo apt update
sudo apt install ros-foxy-desktop   # setup time about 15 min
# Finished to install of Ros2
```
```
source /opt/ros/foxy/setup.bash
ros2 # check command of ros2

export DISPLAY=192.168.1.107:0
# Add source to ~/.bashrc using gedit
sudo nano ~/.bashrc
# go down last and typeing below 
source /opt/ros/foxy/setup.bash
```
- install argcomplete (option)
```
sudo apt install -y python3-pip
pip3 install -U argcomplete
```
- install colcon for ROS2 builds
```
sudo apt install python3-colcon-common-extensions
sudo nano ~/.bashrc  # go down last and typing below 
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash  # ctr+x >> y >> enter
```
- create workspace
```
mkdir ros2_ws && cd ros2_ws
mkdir src
ls   # check directory of src
colcon build
ls    # check directory of install
cd install
ls    # check file of setup.bash
sudo nano ~/.bashrc  # go down last and typing below 
source ~/ro2_ws/install/setup.bash  # ctr+x >> y >> enter 

```

## Install ROS2 Galactic Geochelone on Ubuntu 20.04.1
[ROS2 Galactic Geochelone | Installation | Ubuntu 20.04](https://www.youtube.com/watch?v=B8RIE0obHqw) <br>
[Install ROS2 via Debian Packages](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
### Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings```

```
### Setup Sources
```
apt-cache policy | grep universe
 500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
     release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64
```
```
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS2 Packages
```
sudo apt update
```
```
sudo apt install ros-galactic-desktop
```
```
sudo apt install ros-galactic-ros-base
```
### Enviroment setup
#### Sourcing the setup script
```
source /opt/ros/galactic/setup.bash
```
### Try som examples
in one terminal, source the setup file and then run a C++ 
talker:
```
source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a Python
listener:
```
source /opt/ros/galactic/setup.bash
ros2 run demo_nodes_py listener













## How to in stall Fan Control
1. Connect to the internet.
2. Open "Terminal" in Raspbian.
3. Type the text below in the "Terminal" to initiate installation of Argon ONE PI 4 script.
```
curl https://download.argon40.com/argon1.sh | bash
```
4. Reboot
```
argonone-config
```
```
argonone-uninstall
```
