## This script runs odometry (on one sequence), then runs localization (on any provided sequences)

## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash runtime/run_parallel_test.sh lidar

# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST
ODO_SEQUENCE='boreas-2023-02-15-19-49'
LOC_SEQUENCES=(
'boreas-2023-02-15-20-07'
# 'boreas-2023-02-15-20-24'
# 'boreas-2023-02-15-20-43'
# 'boreas-2023-02-15-21-03'
)

# Get arguments
# MODE=$1         # [odometry, localization]
SENSOR=$1         # [radar, lidar, aeva, radar_lidar]

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}
mkdir -p ${VTRRRESULT}

# maximum number of jobs running in parallel
GROUPSIZE=20

SCRIPT="${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_odometry.sh"
EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_odometry_eval.sh"

# Step 1: Run odometry on the odometry sequence
echo "Running odometry on sequence: $ODO_SEQUENCE"
bash $SCRIPT $ODO_SEQUENCE &>/dev/null
echo "Odometry on $ODO_SEQUENCE completed"

# Step 2: Evaluate the odometry results
echo "Evaluating odometry results for: $ODO_SEQUENCE"
bash $EVAL_SCRIPT $ODO_SEQUENCE
echo "Evaluation of odometry on $ODO_SEQUENCE completed"

# Update the script paths for localization
SCRIPT="${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_localization.sh"
EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_localization_eval.sh"

# Step 3: Run localization using the odometry result as reference
declare -A pids

for seq in ${LOC_SEQUENCES[@]}; do
    echo "Executing command: bash $SCRIPT $ODO_SEQUENCE $seq &>/dev/null &"
    ### command to execute
    bash $SCRIPT $ODO_SEQUENCE $seq &>/dev/null &
    
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

# Step 4: Evaluate localization results
echo "Evaluating localization results"
bash $EVAL_SCRIPT $ODO_SEQUENCE

# Load in param file based on sensor
PARAM_FILE=${ROOTDIR}/runtime/config/${SENSOR}_config.yaml

# Save param file
SAVE_CONFIG=${SENSOR}_odometry_config.yaml
cp ${PARAM_FILE} ${VTRRRESULT}/${ODO_SEQUENCE}/${SAVE_CONFIG}
