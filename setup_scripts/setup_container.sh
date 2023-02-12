# Set variables
export VTRROOT=~/masking_project
export VTRSRC=${VTRROOT}/vtr3
export VTRDEPS=${VTRROOT}/deps
export VTRTEMP=/raid/dli/results_updated_vtr3
export VTRDATA=/raid/dli/boreas
export VTRRROOT=${VTRROOT}/vtr_testing_radar
export VTRRDATA=${VTRDATA}
export ODO_INPUT=boreas-2020-11-26-13-58
export LOC_INPUT=boreas-2020-12-04-14-00
export VTRRRESULT=${VTRTEMP}/radar

# Source setups
source /opt/ros/humble/setup.bash
source ${VTRRROOT}/install/setup.bash

# Activate venv
source ~/masking_project/venv/bin/activate