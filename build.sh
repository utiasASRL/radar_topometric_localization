#!/bin/bash

BUILD_TYPE="Release"
GENERATOR="Unix Makefiles"

SRC_DIR=$(pwd)
EXT_SRC_DIR="${SRC_DIR}/external"
VTR3_SRC_DIR="${SRC_DIR}/vtr3/main"
EXPS_SRC_DIR="${SRC_DIR}/exps"

BUILD_DIR="${SRC_DIR}/cmake-build-${BUILD_TYPE}"
EXT_BUILD_DIR=$BUILD_DIR/external

mkdir -p $BUILD_DIR
mkdir -p $EXT_BUILD_DIR

check_status_code() {
	if [ $1 -ne 0 ]; then
		echo "[VTR3] Failure. Exiting."
		exit 1
	fi
}

# echo "[VTR3] -- [EXTERNAL DEPENDENCIES] -- Generating the cmake project"
# cd ${EXT_BUILD_DIR}
# cmake -G "$GENERATOR" -S $EXT_SRC_DIR -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
# check_status_code $?

# echo "[VTR3] -- [EXTERNAL DEPENDENCIES] -- building CMake Project"
# cmake --build . --config $BUILD_TYPE
# check_status_code $?

echo "[VTR3] -- [VTR3] -- building vtr3 packages"
cd ${VTR3_SRC_DIR}
source /opt/ros/galactic/setup.bash
colcon build --symlink-install \
	--packages-up-to vtr_lidar vtr_radar vtr_radar_lidar \
	--cmake-args \
	-DCMAKE_BUILD_TYPE=${BUILD_TYPE}
