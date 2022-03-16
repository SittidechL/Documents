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
ssh ubuntu@192.168.1.107
username: ubuntu
password: ubuntu
oldpassword:ubuntu
newpassword:********
```
- reboot by it self and one more time to try again SSH
```
ssh ubuntu@192.168.1.107
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

sudo apt update
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
* ros2 to source
```
source /opt/ros/foxy/setup.bash
ros2 # check command of ros2

# Add source to ~/.bashrc using cmd: sudo nano
sudo nano ~/.bashrc   # go down last and typeing below 
source /opt/ros/foxy/setup.bash   # ctr+x >> y >> enter
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
- Try some examples <br>
If you installed | ros-foxy-desktop | above you can try some examples. <br>
In one terminal, source the setup file and then run a C++ talker:
```
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a Python listener:
```
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```
