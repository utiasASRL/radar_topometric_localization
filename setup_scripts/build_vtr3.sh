# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
source /opt/ros/humble/setup.bash  # source the ROS environment
cd $ROOTDIR/external/vtr3/main # Go to where vtr3 is located
colcon build --symlink-install --packages-up-to vtr_lidar vtr_radar vtr_radar_lidar --allow-overriding vtr_common vtr_lidar vtr_logging vtr_radar vtr_radar_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release

cd $ROOTDIR