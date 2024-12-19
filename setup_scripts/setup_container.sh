# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
# Set variables
export VTRROOT=${ROOTDIR}                   # This is required for some internal scripts
export VTRSRC=${ROOTDIR}/external/vtr3
export VTRRROOT=${ROOTDIR}/external/vtr_testing_radar
export VTRRESULT=${ROOTDIR}/results         # POINT THIS TO WHERE YOU WANT TO STORE RESULTS
export VTRRDATA=${ROOTDIR}/data             # POINT THIS TO DATA DIRECTORY

# Source setups
source /opt/ros/humble/setup.bash
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
source ${VTRRROOT}/install/setup.bash

# Activate venv
source ${ROOTDIR}/venv/bin/activate