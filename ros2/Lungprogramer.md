## insatall
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

## Ros2 test
```bash
#terminator#1
ros2 run demo_nodes_py talker

#terminator#2
ros2 run demo_nodes_py listener 
```
