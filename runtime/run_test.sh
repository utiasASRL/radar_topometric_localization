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

# Check if SENSOR is "aeva" and ODO_INPUT starts with "boreas-" or "route-"
if [ "$SENSOR" = "aeva" ] && [[ "$ODO_INPUT" == boreas-* ]]; then
    PARAM_FILE=${VTRRROOT}/src/vtr_testing_${SENSOR}/config/aeva_boreas.yaml
    TYPE="aeva_boreas"
    export VTRRDATA=${BOREAS}
elif  [ "$SENSOR" = "aeva" ] && [[ "$ODO_INPUT" == 2024-* ]]; then
    echo "AEVA II"
    PARAM_FILE=${VTRRROOT}/src/vtr_testing_${SENSOR}/config/aeva_boreas.yaml
    TYPE="aeva_boreas"
elif [ "$SENSOR" = "aeva" ] && [[ "$ODO_INPUT" == route* ]]; then
    PARAM_FILE=${VTRRROOT}/src/vtr_testing_${SENSOR}/config/aeva_hq.yaml
    TYPE="aeva_hq"
    export VTRRDATA=${AEVAHQ}
fi

echo "PARAM FILE IS ${PARAM_FILE}"

# Save param file
SAVE_CONFIG=${SENSOR}_${MODE}_config.yaml
cp ${PARAM_FILE} ${VTRRRESULT}/${ODO_INPUT}/${SAVE_CONFIG}

# Call corresponding script from vtr_testing_radar
if [ "$1" = "odometry" ]; then
    bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_odometry.sh ${ODO_INPUT} ${TYPE}
else
    bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_localization.sh ${ODO_INPUT} ${LOC_INPUT} ${TYPE}
fi