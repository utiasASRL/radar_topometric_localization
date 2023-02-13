# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory

# Rebuild vtr3
cd $ROOTDIR
source setup_scripts/build_vtr3.sh

# Rebuild vtr_testing_radar
cd $ROOTDIR
source setup_scripts/build_vtr_testing_radar.sh