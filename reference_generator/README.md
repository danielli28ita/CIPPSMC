# px4_reference_generator

作者：李丹钰

ROS 2 Humble；px4_msgs release/1.15。

参数：`config/`。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select px4_reference_generator
source install/setup.bash
ros2 launch px4_reference_generator reference_generator.launch.py
```
