# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
# Set variables
export VTRSRC=$ROOTDIR/external/vtr3
export VTRRROOT=$ROOTDIR/external/vtr_testing_radar
export VTRRESULT=$ROOTDIR/results           # POINT THIS TO WHERE YOU WANT TO STORE RESULTS
export VTRRDATA=$ROOTDIR/data                # POINT THIS TO DATA DIRECTORY
export VTRRDATA=/raid/dli/boreas             # DELETE THIS
export VTRRRESULT=${VTRRESULT}/radar        # POINT THIS TO DIRECTORY INSIDE RESULT DIR

mkdir -p ${VTRRRESULT}

# Set odometry and localization inputs
export ODO_INPUT=boreas-2020-11-26-13-58    # FROM BOREAS DATASET, CHANGE THIS POTENTIALLY
export LOC_INPUT=boreas-2020-12-04-14-00    # FROM BOREAS DATASET, CHANGE THIS POTENTIALLY

# Source setups
source /opt/ros/humble/setup.bash
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
source ${VTRRROOT}/install/setup.bash

# Activate venv
source $ROOTDIR/venv/bin/activate