## Create Nodes
![image](https://user-images.githubusercontent.com/60011264/165297963-9c532a32-2bb4-41fe-8e23-bdb294b765dd.png)
```bash
cd /ros2_ws/scr/my_py_pkg/my_py_pkg/
touch my_first_node.py
gedit my_first_node.py
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
