source setup_container.sh

# Rebuild vtr3
source /opt/ros/humble/setup.bash  # source the ROS environment
cd ${VTRSRC}/main
colcon build --symlink-install --packages-up-to vtr_lidar vtr_radar vtr_radar_lidar --allow-overriding vtr_common vtr_lidar vtr_logging vtr_radar vtr_radar_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release

# Rebuild vtr_testing_radar
source /opt/ros/humble/setup.bash
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
cd ~/masking_project/vtr_testing_radar # go to where this repo is located
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cd ~/masking_project

source setup_container.sh