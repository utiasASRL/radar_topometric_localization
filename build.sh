#!/bin/bash

BUILD_TYPE="Release"
GENERATOR="Unix Makefiles"

export VTRROOT=$(pwd)
export VTRSRC=${VTRROOT}/vtr3
export VTRTEMP=${VTRROOT}/results

check_status_code() {
	if [ $1 -ne 0 ]; then
		echo "[VTR3] Failure. Exiting."
		exit 1
	fi
}

echo "[VTR3] -- [VTR3] -- building vtr3 packages"
cd ${VTRSRC}/main
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
	--packages-up-to vtr_lidar vtr_radar vtr_radar_lidar \
	--cmake-args \
	-DCMAKE_BUILD_TYPE=${BUILD_TYPE}
check_status_code $?

echo "[EXPS] -- [EXPS] -- building exps package"
cd ${EXPSSRC}
source ${VTRSRC}/main/install/setup.bash
colcon build --symlink-install \
	--cmake-args \
	-DCMAKE_BUILD_TYPE=${BUILD_TYPE}
check_status_code $?
