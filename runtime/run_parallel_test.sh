## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_parallel_test.sh localization radar

# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST IN PARALLEL FOR EITHER MODE
if [ "$1" = "odometry" ]; then
    # Odometry sequences, SET THESE YOURSELF
    SEQUENCES=(
    'boreas-2020-11-26-13-58'
    'boreas-2020-12-04-14-00'
    'boreas-2021-01-26-10-59'
    'boreas-2021-02-09-12-55'
    'boreas-2021-03-09-14-23'
    'boreas-2021-06-29-18-53'
    'boreas-2021-09-08-21-00'
    )
else
    # Odometry reference for localization, SET THIS YOURSELF
    REFERENCE='boreas-2020-11-26-13-58'
    # Localization sequences, SET THESE YOURSELF
    SEQUENCES=(
    'boreas-2020-12-04-14-00'
    'boreas-2021-01-26-10-59'
    'boreas-2021-02-09-12-55'
    'boreas-2021-03-09-14-23'
    'boreas-2021-06-29-18-53'
    'boreas-2021-09-08-21-00'
    )
fi

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]

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
    if [ "$1" = "odometry" ]; then
        echo "Executing command: bash $SCRIPT $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $seq &>/dev/null &
    else
        echo "Executing command: bash $SCRIPT $REFERENCE $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $REFERENCE $seq &>/dev/null &
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
    echo "Executing command: bash $EVAL_SCRIPT $REFERENCE"
    bash $EVAL_SCRIPT $REFERENCE
fi