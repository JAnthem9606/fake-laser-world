# 🧪 fake_laser_worlds

A ROS 2 package that simulates laser scan (`sensor_msgs/LaserScan`) data in various synthetic 2D environments — perfect for testing SLAM, navigation, or laser-based perception without a physical LiDAR sensor.

---

## 🚀 Features

- ✅ Publishes fake laser scans to the `/scan` topic
- ✅ Multiple environment types:
  - `maze`, `maze2`
  - `room`, `wall`, `corridor`
  - `U-shape`, `L-shape`
  - `boxes`, `furniture`
  - `ellipse`, `sine` patterns
- ✅ Ready-to-use ROS 2 nodes
- ✅ Easy integration with RViz for visualization
- ✅ Works with SLAM packages like `slam_toolbox`, `gmapping`, etc.

---

## 📁 Package Contents

Each environment is implemented as a separate Python script under `fake_laser_worlds`:

```
fake_laser_boxes.py
fake_laser_corridor.py
fake_laser_ellipse.py
fake_laser_furniture.py
fake_laser_lshape.py
fake_laser_maze.py
fake_laser_maze2.py
fake_laser_room.py
fake_laser_sine.py
fake_laser_ushape.py
fake_laser_wall.py
```

Each node publishes a 360-degree simulated `LaserScan` based on geometry logic and includes a static transform (`base_link → laser_frame`).

---

## 📦 Installation

### 1. Clone into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/JAnthem9606/fake-laser-world.git fake_laser_worlds
```

### 2. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select fake_laser_worlds
source install/setup.bash
```

---

## ▶️ Usage

Launch any fake laser environment using:

```bash
ros2 run fake_laser_worlds fake_laser_maze
```

or replace `fake_laser_maze` with any of the other node names.

---

## 🖥️ Visualize in RViz

```bash
rviz2
```

In RViz:
- Set **Fixed Frame** to: `odom`
- Add **LaserScan** and set topic: `/scan`

---

## 🧰 Use Cases

- SLAM testing without real sensors
- Navigation stack development (e.g., move_base, nav2)
- Robot perception prototyping
- Educational and teaching demos

---

## 🤝 Contributing

New pattern ideas, bug fixes, or improvements are welcome! Feel free to open issues or pull requests.

---

## 📄 License

[MIT License](LICENSE) — feel free to use, modify, and share.

---

## 🙌 Acknowledgments

Created and maintained by [@JAnthem9606](https://github.com/JAnthem9606)  
If you use this in your work, a star ⭐ on GitHub would be appreciated!
