## Create package
![image](https://user-images.githubusercontent.com/60011264/165297046-b9888ebb-7057-41da-b126-d0a7ec436b24.png)
```bash
cd ros2_ws/scr/
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
cd ..
source install/local_setup.bash
colcon build
#or by package
colcon build --packages-select my_py_pkg

```
