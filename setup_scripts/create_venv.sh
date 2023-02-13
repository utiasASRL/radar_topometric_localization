# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
cd $ROOTDIR
virtualenv venv
source venv/bin/activate

cd $ROOTDIR/external/pyboreas
pip install -e .
pip install pyyaml
pip install pandas

deactivate