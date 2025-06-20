colcon build --packages-select champ mujoco_msgs haptiquad_msgs --event-handlers desktop_notification-
source install/setup.bash
colcon build --packages-skip haptiquad_ros2 mujoco description --event-handlers desktop_notification-
source install/setup.bash
colcon build --symlink-install --packages-select description mujoco --event-handlers desktop_notification-
source install/setup.bash
colcon build --cmake-force-configure --packages-select haptiquad_ros2