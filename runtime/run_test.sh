## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_test.sh localization radar boreas-2020-11-26-13-58 boreas-2020-12-04-14-00

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]
ODO_INPUT=$3    # Boreas sequence
LOC_INPUT=$4    # Boreas sequence, not used if mode=odometry

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}
mkdir -p ${VTRRRESULT}

# Load in param file based on sensor
PARAM_FILE=${ROOTDIR}/runtime/config/${SENSOR}_config.yaml

# Call corresponding script from vtr_testing_radar
if [ "$1" = "odometry" ]; then
    bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_odometry.sh ${ODO_INPUT} ${PARAM_FILE}
else
    bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_localization.sh ${ODO_INPUT} ${LOC_INPUT} ${PARAM_FILE}
fi