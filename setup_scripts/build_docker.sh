# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
cd $ROOTDIR
docker build --rm -t radar_loc_$(whoami) \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} .