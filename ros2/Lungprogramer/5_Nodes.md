## Create Nodes
![image](https://user-images.githubusercontent.com/60011264/165297963-9c532a32-2bb4-41fe-8e23-bdb294b765dd.png)
```bash
cd /ros2_ws/scr/my_py_pkg/my_py_pkg/
touch my_first_node.py
gedit my_first_node.py
chmod +x my_first_node.py
```
  ```bash
  #!/usr/bin/env python3
  import rclpy
  from rclpy.node import Node

  class MyNode(Node):
    def __init__(self):
      super().__init__("py_test")
      self.get_logger().info("Hello ROS2")


  def main(args=None):
      rclpy.init(args=args)
      node = MyNode()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == "__main__":
      main() 
  ```
```bash
cd /ros2_ws/scr/my_py_pkg/my_py_pkg/
./my_first_node.py
```
## Create py_node
```bash
cd /ros2_ws/scr/my_py_pkg/
gedit setup.py
#add inside
"py_node = my_py_pkg.my_first_node:main"
```
![image](https://user-images.githubusercontent.com/60011264/165299815-270337e3-e09d-4518-98ad-b3bc1cd59860.png)
```bash
cd /ros2_ws/
colcon build --packages-select my_py_pkg
cd ros2_ws/install/my_py_pkg/lib/my_py_pkg
./py_node
```
```bash
source .bashrc
ros2 run my_py_pkg py_node
```
## time_callback
```bash
cd /ros2_ws/scr/my_py_pkg/my_py_pkg/
gedit my_first_node.py
```
![image](https://user-images.githubusercontent.com/60011264/165302349-184dfb04-c71e-4af3-a50e-b46cff0077c9.png)
