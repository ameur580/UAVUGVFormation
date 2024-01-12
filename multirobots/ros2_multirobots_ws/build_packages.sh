colcon build --symlink-install --event-handlers console_direct+ --packages-select mr_interfaces
colcon build --symlink-install --event-handlers console_direct+ --packages-select mr_clock
colcon build --symlink-install --event-handlers console_direct+ --packages-select mr_aggregator
. install/setup.bash
colcon build --symlink-install --event-handlers console_direct+ --packages-select mr_oracle
. install/setup.bash
colcon build --symlink-install --event-handlers console_direct+ --packages-select epuck_controller
