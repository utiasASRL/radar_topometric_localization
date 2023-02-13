# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
# Need to additionally set VTRSRC variable
export VTRSRC=$ROOTDIR/external/vtr3
source /opt/ros/humble/setup.bash
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
cd $ROOTDIR/external/vtr_testing_radar # go to where vtr_testing_radar is located
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cd $ROOTDIR