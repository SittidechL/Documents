## 🛠️ ติดตั้งและรัน rplidar_ros บน ROS2 Humble

### 1️⃣ ตรวจสอบว่ามีแพ็กเกจอยู่จริงหรือไม่

```bash
ls ~/rplidar_ws/src
```

ต้องเห็นโฟลเดอร์:

```
rplidar_ros
```

ถ้าไม่มีให้ clone ใหม่:

```bash
cd ~/rplidar_ws/src
git clone -b humble https://github.com/Slamtec/rplidar_ros.git
```

---

### 2️⃣ Clean build และ Build ใหม่

```bash
cd ~/rplidar_ws
rm -rf build install log
colcon build --symlink-install
```

> หากใช้ Raspberry Pi แล้ว build ช้า สามารถเพิ่ม swap ได้

---

### 3️⃣ Source Workspace

```bash
cd ~/rplidar_ws
source install/setup.bash
```

ทดสอบว่า ROS เห็นแพ็กเกจหรือไม่:

```bash
ros2 pkg list | grep rplidar
```

---

### 4️⃣ รัน Node

```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser
```
