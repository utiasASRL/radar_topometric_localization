## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!

## Before running this script, please specify the user inputs below.
## example usage: run_parallel_test_multi_extractor.sh

# USER INPUT: Select the point-cloud extractors you wish to test
CONFIGS=(
# 'cacfar'
# 'bfar'
'kstrongest'
# 'cen2018'
# 'cen2019'
# 'oscfar'
# 'cago_cfar'
# 'caso_cfar'
# 'msca_cfar'
# 'tm_cfar'
# 'is_cfar'
# 'vi_cfar'
# 'cfear_kstrong'
)


# USER INPUT: Select the sequeces you wish to test
SEQUENCES=(
# 'boreas-2021-10-05-15-35'
'boreas-2021-10-15-12-35' 
# 'boreas-2021-10-22-11-36' 
# 'boreas-2021-10-26-12-35' 
# 'boreas-2021-11-02-11-16' 
# 'boreas-2021-11-06-18-55' 
# 'boreas-2021-11-14-09-47' 
# 'boreas-2021-11-16-14-10' 
# 'boreas-2021-11-23-14-27' 
# 'boreas-2021-11-28-09-18'
)

# USER INPUT: Modify this file to control the min and max parameter values for the respective extractors you wish to sweep 
PARAM_FILE=${VTRRROOT}/src/vtr_testing_radar/config/boreas.yaml

# USER INPUT: Specify the number of extractor-sequence pairs you wish to sweep in parallel
GROUPSIZE=2

# IMPORTANT NOTE: You must also specify the variable 'max_process_count' in parameter_search.py.
# This value will dictate the number of processes that run for every extractor sequence pair.

# Total Processes in Parallel = GROUPSIZE * max_process_count (from parameter_search.py)


MODE="odometry"
SENSOR="radar"

declare -A pids

for config in ${CONFIGS[@]}; do
    for seq in ${SEQUENCES[@]}; do

    echo "Running Extractor, Sequence Pair: $config , $seq"
    python ${ROOTDIR}/extractor_tuning/parameter_search.py --config ${config} --config_path ${PARAM_FILE} --mode ${MODE} --sensor ${SENSOR} --seq ${seq}
    # python ${ROOTDIR}/extractor_tuning/parameter_search.py --config ${config} --config_path ${PARAM_FILE} --mode ${MODE} --sensor ${SENSOR} --seq ${seq} &

    sleep 1
    
    pids[${seq}]=$!
    # pid=$!
    # pids[${pid}]=$pid

    # echo "Number of active processes: ${#pids[@]}"
    # wait for all pids to finish if reached group size
    if [[ ${#pids[@]} -ge ${GROUPSIZE} ]]; then
    for key in ${!pids[@]}; do
        wait ${pids[${key}]}
        echo "Process ${key} finished with return code ${?}"
        unset pids[${key}]
    done
    fi

    done
done
