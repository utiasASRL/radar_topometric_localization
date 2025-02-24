## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_parallel_test.sh localization radar

# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST IN PARALLEL FOR EITHER MODE
if [ "$1" = "odometry" ]; then
    # Odometry sequences, SET THESE YOURSELF
    SEQUENCES=(
    'boreas-2023-02-15-19-49'
    )
else
    # Odometry reference for localization, SET THIS YOURSELF
    REFERENCE='boreas-2023-02-15-19-49'
    # Localization sequences, SET THESE YOURSELF
    SEQUENCES=(
    'boreas-2023-02-15-20-07'
    'boreas-2023-02-15-20-24'
    'boreas-2023-02-15-20-43'
    # 'boreas-2023-02-15-21-03' # different route
    )
fi

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]

# Load in param file based on sensor
PARAM_FILE=${ROOTDIR}/runtime/config/${SENSOR}_config.yaml

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}
mkdir -p ${VTRRRESULT}

# maximum number of jobs running in parallel
GROUPSIZE=20

SCRIPT="${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_${MODE}.sh"
EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_${MODE}_eval.sh"

declare -A pids

# Run tests in parallel
for seq in ${SEQUENCES[@]}; do
    # Save param file
    SAVE_CONFIG=${SENSOR}_${MODE}_config.yaml
    cp ${PARAM_FILE} ${VTRRRESULT}/$seq/${SAVE_CONFIG}

    if [ "$1" = "odometry" ]; then
        echo "Executing command: bash $SCRIPT $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $seq &>/dev/null &
    else
        echo "Executing command: bash $SCRIPT $REFERENCE $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $REFERENCE $seq ${TYPE} &>/dev/null &
    fi

    pids[${seq}]=$!
    # wait for all pids to finish if reached group size
    if [[ ${#pids[@]} -ge ${GROUPSIZE} ]]; then
    for key in ${!pids[@]}; do
        wait ${pids[${key}]}
        echo "Process ${key} finished with return code ${?}"
        unset pids[${key}]
    done
    fi
done

for key in ${!pids[@]}; do
  wait ${pids[${key}]}
  echo "Process ${key} finished with return code ${?}"
  unset pids[${key}]
done

# Evaluate results from tests
if [ "$1" = "odometry" ]; then
    for seq in ${SEQUENCES[@]}; do
    echo "Executing command: bash $EVAL_SCRIPT $seq"
    bash $EVAL_SCRIPT $seq
    done
else
    echo "Executing command: bash $EVAL_SCRIPT $REFERENCE $TYPE"
    bash $EVAL_SCRIPT $REFERENCE
fi