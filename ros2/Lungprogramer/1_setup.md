## install ROS3 via Debian Packages (Raspberry Pi)
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
* Set local
  --------
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
* Setup Source
  -----------
```bash
apt-cache policy | grep universe
 500 http://us.archive.ubuntu.com/ubuntu focal/universe amd64 Packages
     release v=20.04,o=Ubuntu,a=focal,n=focal,l=Ubuntu,c=universe,b=amd64
```
If you don’t see an output line like the one above, then enable the Universe repository with these instructions.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 apt repository to your system. First authorize our GPG key with apt.
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to you sources list
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
* Install ROS2 packages
  --------
```bash
sudo apt update
```
Desktop install (Recommended):ROS, RViz, demos, tutorials.
```bash
sudo apt install ros-galactic-desktop
```
Ros-Base install(bare bones): communication libraries, message packages, command line tools. No GUI tools
.
```bash
sudo apt install ros-galactic-base
```
* Enviroment setup
  * Sourcing the setup script
  ```bash
  source /opt/ros/galactic/setup.bash
  ```
  * Try some examples
    ```bash
    #terminator#1
    source /opt/ros/galactic/setup.bash
    ros2 run demo_nodes_py talker
    ```
    ```bash
    #terminator#2
    source /opt/ros/galactic/setup.bash
    ros2 run demo_nodes_py listener 
    ```
![image](https://user-images.githubusercontent.com/60011264/165293801-662c5aad-030e-492e-a70e-cce5e44ab5a5.png)
