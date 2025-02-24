## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_eval.sh localization radar boreas-2020-11-26-13-58

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]
ODO_INPUT=$3    # Boreas sequence
TYPE=$4         # For aeva only

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}

# Check if SENSOR is "aeva" and ODO_INPUT starts with "boreas-" or "route-"
if [ "$SENSOR" = "aeva" ] && [[ "$ODO_INPUT" == boreas-* ]]; then
    TYPE="aeva_boreas"
    export VTRRDATA=${BOREAS}
elif [ "$SENSOR" = "aeva" ] && [[ "$ODO_INPUT" == 20* ]]; then
    echo "AEVA II"
    TYPE="aevaii_boreas"
elif [ "$SENSOR" = "aeva" ] && [[ "$ODO_INPUT" == route* ]]; then
    TYPE="aeva_hq"
    export VTRRDATA=${AEVAHQ}
fi

# Call corresponding script from vtr_testing_radar
result_dir="${VTRRRESULT}/${ODO_INPUT}"
loc_inputs=($(ls ${result_dir} | grep '^20' | grep -v ${ODO_INPUT}))

echo "loc inputs: ${loc_inputs[@]}"

for LOC_INPUT in "${loc_inputs[@]}"; do
    bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_${MODE}_eval.sh ${ODO_INPUT} ${TYPE} ${LOC_INPUT}
done