## Publisher
```bash
cd ros2_ws/scr/my_py_pkg/my_py_pkg
touch robot_news_station.py
chmod +x robot_news_station.py
gedit robot_news_station.py
```
  ```bash
  #!/usr/bin/env python3
  import rclpy
  from rclpy.node import Node

  from example_interfaces.msg import String

  class RobotNewsStationNode(Node):
    def __init__(self):
      super().__init__("robot_news_station")

      self.robot_name_ = "C3P0"
      self.publisher_ = self.create_publisher(String, "robot_news", 10)
      self.timer_ = self.create_timer(0.5, self.publish_news)
      self.get_logger().info("Robot New Station has been started")

    def publish_news(self):
      msg = String()
      msg.data = "Hi, this is " + \
        str(self.robot_name_) + " from the robot news station."
      self.publisher_.publish(msg)

  def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

  if __name__ == "__main__":
    main()
  ```
```bash
#setup.py
"robot_news_station = my_py_pkg.robot_news_station:main"
```
```bash
#package.xml
<depend>example_interfaces</depend>
```
```bash
cd ros2_ws
colcon build --packages-select my_py_pkg --symlink-install
ros2 run my_py_pkg robot_news_station
```
```bash
ros2 node list
ros2 topic list
ros2 topic echo /robot_news
```
## Subscriber
```bash
touch smartphone.py
chmod +x smartphone.py
```
```bash
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class SmartphoneNode(Node):
	def __init__(self):
		super().__init__("smartphone")
		
		self.robot_name_ = "C3P0"
		self.publisher_ = self.create_subscription(
			String, "robot_news",self.callback_robot_news, 10)
		self.get_logger().info("Smartphone has been started.")
		
	def callback_robot_news(self, msg):
		self.get_logger().info(msg.data)

def main(args=None):
	rclpy.init(args=args)
	node = SmartphoneNode()
	rclpy.spin(node)
	rclpy.shutdown()
	
if __name__ == "__main__":
	main()
```
```bash
#setup.py
"smartphone = my_py_pkg.smartphone:main"
```
```bash
ros2 run my_py_pkg smartphone
```
