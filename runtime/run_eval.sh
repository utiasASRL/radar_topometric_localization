## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_eval.sh localization radar boreas-2020-11-26-13-58

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]
ODO_INPUT=$3    # Boreas sequence

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}

# Load in param file based on sensor
PARAM_FILE=${ROOTDIR}/runtime/config/${SENSOR}_config.yaml

# Call corresponding script from vtr_testing_radar
bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_${MODE}_eval.sh ${ODO_INPUT} ${PARAM_FILE}