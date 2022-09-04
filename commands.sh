echo "This script contains instructions to run all tests, do not run this script directly."
exit 1

SRC_DIR=$(pwd)
export VTRSRC=${SRC_DIR}/vtr3
export EXPSSRC=${SRC_DIR}/exps
export DATASET=${HOME}/ASRL/data/boreas/sequences
export RESULT=${HOME}/ASRL/temp/radar_loc

## First launch RViz for visualization
source /opt/ros/galactic/setup.bash               # source the ROS environment
ros2 run rviz2 rviz2 -d ${EXPSSRC}/rviz/lidar.rviz # launch rviz

## Then in another terminal, launch rqt_reconfigure for control
## current supported dynamic reconfigure parameters: control_test.play and controL_test.delay_millisec
source /opt/ros/galactic/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure

## Now start another terminal and run testing scripts ####
source ${EXPSSRC}/install/setup.bash

# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
SENSOR=radar_lidar
ODO_INPUT=boreas-2020-11-26-13-58
LOC_INPUT=boreas-2021-01-26-10-59
MODE=localization

# MODE=preprocessing or odometry
source ${EXPSSRC}/install/setup.bash
ros2 run exps exps_${SENSOR}_${MODE} \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${EXPSSRC}/config/${SENSOR}_config.yaml \
  -p output_dir:=${RESULT}/${SENSOR}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT}

# MODE=localization
rm -r ${RESULT}/${SENSOR}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${RESULT}/${SENSOR}/${ODO_INPUT}/${ODO_INPUT} ${RESULT}/${SENSOR}/${ODO_INPUT}/${LOC_INPUT}
source ${EXPSSRC}/install/setup.bash
ros2 run exps exps_${SENSOR}_${MODE} \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${EXPSSRC}/config/${SENSOR}_config.yaml \
  -p output_dir:=${RESULT}/${SENSOR}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}
