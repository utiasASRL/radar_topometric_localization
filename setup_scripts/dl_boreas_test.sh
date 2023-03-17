#!/bin/bash
aws s3 sync s3://boreas/boreas-2020-11-26-13-58 $ROOTDIR/data/boreas-2020-11-26-13-58 --exclude '*'  --include 'radar/*' --include 'applanix/*' --include 'calib/*' --no-sign-request