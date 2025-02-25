# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
# Need to additionally set VTRSRC variable
export VTRSRC=$ROOTDIR/external/vtr3
source /opt/ros/humble/setup.bash
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
cd $ROOTDIR/external/boreas_vtr_wrapper # go to where vtr_boreas_vtr_wrappertesting_radar is located
MAKEFLAGS="-j$(($(nproc --all) / 4 + 1))" colcon build --packages-select vtr_testing_aeva --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cd $ROOTDIR