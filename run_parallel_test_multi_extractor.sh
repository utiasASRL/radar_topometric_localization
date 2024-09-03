## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_parallel_test.sh localization radar
## This script also assumes that you have selected the test sequences you wish to use in run_parallel_test.sh

# USER INPUT: SELECT THE CONFIGS YOU WISH TO TEST FOR PREPROCESSING & ODOMETRY
CONFIGS=(
# 'landmark_extraction'
# 'surf'
# 'modified_cacfar_power'
# 'bfar_pure'
# 'kstrongest'
# 'cen2018'
# 'cen2019'
# 'cacfar'
# 'oscfar'
# 'cago_cfar'
'caso_cfar'
# 'msca_cfar'
# 'tm_cfar'
# 'is_cfar'
# 'vi_cfar'
# 'cfear_kstrong'
)


# #OLDEST
# SEQUENCES=(
#     'boreas-2020-11-26-13-58'
#     # 'boreas-2021-01-26-10-59'
#     # 'boreas-2021-03-09-14-23'
#     )

# OLD Training
# SEQUENCES=(
# # 'boreas-2020-12-01-13-26' 
# 'boreas-2021-03-02-13-38' 
# # 'boreas-2021-04-29-15-55' 
# # 'boreas-2021-06-17-17-52' 
# # 'boreas-2021-08-05-13-34' 
# # 'boreas-2021-09-07-09-35'
# )

# # #OLD Testing
# SEQUENCES=( 
# 'boreas-2020-12-04-14-00' 
# 'boreas-2021-01-26-10-59' 
# 'boreas-2021-02-09-12-55'
# 'boreas-2021-03-09-14-23' 
# 'boreas-2021-06-29-18-53'
# 'boreas-2021-09-08-21-00'
# )


# NEW Training
SEQUENCES=(
'boreas-2021-10-05-15-35'
'boreas-2021-10-15-12-35' 
'boreas-2021-10-22-11-36' 
'boreas-2021-10-26-12-35' 
)

# # NEW Testing
# SEQUENCES=(
# 'boreas-2021-11-02-11-16' 
# 'boreas-2021-11-06-18-55' 
# 'boreas-2021-11-14-09-47' 
# 'boreas-2021-11-16-14-10' 
# 'boreas-2021-11-23-14-27' 
# 'boreas-2021-11-28-09-18'
# )

PARAM_FILE=${VTRRROOT}/src/vtr_testing_radar/config/boreas.yaml
comment=" # choose detector \in (kstrongest, cen2018, cacfar, oscfar, modified_cacfar)"

SCRIPT="${VTRROOT}/runtime/run_parallel_test.sh"
MODE="odometry"
SENSOR="radar"

# maximum number of jobs running in parallel
GROUPSIZE=4
# GROUPSIZE=3
declare -A pids

for config in ${CONFIGS[@]}; do
    for seq in ${SEQUENCES[@]}; do

    echo "Running Config, Sequence Pair: $config , $seq"
    # python ${ROOTDIR}/parameter_search.py --config ${config} --config_path ${PARAM_FILE} --mode ${MODE} --sensor ${SENSOR} --seq ${seq}
    python ${ROOTDIR}/parameter_search.py --config ${config} --config_path ${PARAM_FILE} --mode ${MODE} --sensor ${SENSOR} --seq ${seq} &

    sleep 2
    
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
done
