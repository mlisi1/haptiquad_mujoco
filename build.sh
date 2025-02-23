colcon build --packages-select champ --event-handlers desktop_notification-
source install/setup.bash
colcon build --packages-skip momobs_ros2 mujoco description --event-handlers desktop_notification-
source install/setup.bash
colcon build --symlink-install --packages-select description mujoco --event-handlers desktop_notification-
source install/setup.bash
colcon build --cmake-force-configure --packages-select momobs_ros2