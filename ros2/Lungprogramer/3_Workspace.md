## Create Workspace
```bash
mkdir ros2_ws && cd ros2_ws
mkdir scr
colcon build
source install/local_setup.bash
gedit ~/.bashrc
# add below
source ~/ros2_ws/install/setup.bash
```
