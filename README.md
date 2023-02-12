# radar_topometric_localization

Paper: [Are We Ready for Radar to Replace Lidar in All-Weather Mapping and Localization?](https://ieeexplore.ieee.org/abstract/document/9835037)

Dataset: [Boreas](https://www.boreas.utias.utoronto.ca/#/)

# Citation

```bibtex
@article{burnett_ral22,
  author={Burnett, Keenan and Wu, Yuchen and Yoon, David J. and Schoellig, Angela P. and Barfoot, Timothy D.},
  journal={IEEE Robotics and Automation Letters},
  title={Are We Ready for Radar to Replace Lidar in All-Weather Mapping and Localization?},
  year={2022},
  volume={7},
  number={4},
  pages={10328-10335},
  doi={10.1109/LRA.2022.3192885}
}
```

# Installation

## Setup VTR3 Directories

Create the following directories in your local filesystem. Later they will be mapped to the docker container.

```Bash
export VTRROOT=~/ASRL  # (INTERNAL default) root directory of VTR3
# you can change the following directories to anywhere appropriate
export VTRSRC=${VTRROOT}/vtr3        # source code of VTR3
export VTRRESULT=${VTRROOT}/results  # result directory

mkdir -p ${VTRSRC} ${VTRRESULT}
```

Reference: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide

## Download VTR3 Source Code

Also to your local filesystem, so that you don't have to access them from within the docker container.

```Bash
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git checkout 6d64daa  # commit hash specified by radar_topometric_localization
git submodule update --init --remote
```

Reference: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide

## Download vtr_testing_radar

This package contains testing code for lidar and radar pipeline. Download it do your local filesystem.

```Bash
cd ${VTRROOT}
git clone git@github.com:cheneyuwu/vtr_testing_radar.git
cd vtr_testing_radar
git checkout aeva_cov_interp
```

## Download pyboreas for evaluation

```Bash
cd ${VTRROOT}
git clone git@github.com:utiasASRL/pyboreas.git
cd pyboreas
git checkout localization_eval
```

## Build VTR3 Docker Image

This builds a image that has all dependencies installed.

```Bash
cd ${VTRSRC}
docker build -t vtr3_<your_name> \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} .
```

Reference: https://github.com/utiasASRL/vtr3/wiki/EXPERIMENTAL-Running-VTR3-from-a-Docker-Container

## Start the VTR3 Docker container

Install nvidia docker runtime first: https://nvidia.github.io/nvidia-container-runtime/

```Bash
docker run -dit --rm --name vtr3 \
  --privileged \
  --network=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}:${HOME}:rw \
  -v ${HOME}/ASRL:${HOME}/ASRL:rw vtr3_<your_name>
```

FYI: to start a new terminal with the existing container: `docker exec -it vtr3 bash`

Reference: https://github.com/utiasASRL/vtr3/wiki/EXPERIMENTAL-Running-VTR3-from-a-Docker-Container

## Build and Install VT&R3

Start a new terminal and enter the container

```Bash
source /opt/ros/humble/setup.bash  # source the ROS environment
cd ${VTRSRC}/main
colcon build --symlink-install --packages-up-to vtr_lidar vtr_radar vtr_radar_lidar
```

wait until it finishes.

## Build and Install vtr_testing_radar (this package)

```Bash
source /opt/ros/humble/setup.bash
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
cd ~/ASRL/vtr_testing_radar # go to where this repo is located
colcon build --symlink-install
```

wait until it finishes.

Note that whenever you change any code in the vtr3 repo, you need to re-compile and re-install, do this by re-running the `colcon build ....` command for both vtr3 and then vtr_testing. Always wait until build process on vtr3 finishes before running the build command for vtr_testing.

## Create a python venv to install pyboreas

Within the running container, create a virtual environment at `${VTRROOT}`

```Bash
cd ${VTRROOT}
virtualenv venv
source venv/bin/activate  # activate this environment
```

Install pyboreas `localization_eval` branch

```Bash
cd <where you downloaded pyboreas to>
pip install -e .
pip install pyyaml
pip install pandas
```

# Running Experiments

## Visualization (Work in Progress)

First launch RVIZ for visualization:

```Bash
source /opt/ros/humble/setup.bash               # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/radar.rviz # launch rviz
```

Then in another terminal, launch `rqt_reconfigure` for control. Currently supported dynamic reconfigure parameters: `control_test.play` and `control_test.delay_millisec`

```Bash
source /opt/ros/humble/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
```

## Odometry (Teach) and Localization (Repeat)

```Bash
export VTRRROOT=${VTRROOT}/vtr_testing_radar # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRRESULT}/radar/boreas    # default result location
mkdir -p ${VTRRRESULT}
```

```Bash
source ${VTRRROOT}/install/setup.bash
# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2020-11-26-13-58
LOC_INPUT=boreas-2021-01-26-10-59
```

Odometry:
```Bash
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_odometry.sh ${ODO_INPUT}
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_odometry_eval.sh ${ODO_INPUT}
```

Localization:
```Bash
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_localization.sh ${ODO_INPUT} ${LOC_INPUT}
# Evaluation:
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_localization_eval.sh ${ODO_INPUT}
```


## Running Experiments in Parallel

Assuming you want to run odometry or localization for all test sequences in parallel.

Inside the `script` folder of all three testing packages (`vtr_testing_<...>`), you can find the following two bash script:

- `parallel_test_odometry.sh`
- `parallel_test_localization.sh`


All you need to do is run one of the above bash scripts **inside the contaner**. 

```
bash <path to parallel_test_odometry.sh or parallel_test_localization.sh>
```

For example,

```
bash ${VTRRROOT}/src/vtr_testing_radar/script/parallel_test_localization.sh
```

Then monitor progress by going to the log file of each test.

The log file should be located at

`~/ASRL/temp/[radar, lidar, radar_lidar]/boreas/<boreas-2020-11-26-13-58>/<boreas-2020-11-26-13-58>/<some name based on time>.log`

Understand what these scripts do:

Using `parallel_test_odometry.sh` from `src/vtr_testing_radar/script` as an example, the script does the following:

1. Define sequences we need to run for odometry

```
# odometry sequences
SEQUENCES=(
  'boreas-2020-11-26-13-58'  # Note this is the localization reference run, you must run this in order to run localization tests
  'boreas-2020-12-04-14-00'
  'boreas-2021-01-26-10-59'
  'boreas-2021-02-09-12-55'
  'boreas-2021-03-09-14-23'
  'boreas-2021-04-22-15-00'
  'boreas-2021-06-29-18-53'
  'boreas-2021-06-29-20-43'
  'boreas-2021-09-08-21-00'
)
```

2. Set max number of sequences to run in parallel

```
# maximum number of jobs running in parallel
GROUPSIZE=20
```

3. Setup up directories

These directories are defined using the environment variables in `Setup VTR3 Directories` section.

I suggest you don't change them.

For `VTRRDATA`, it is supposed to be the directory that contains all boreas sequences (i.e. `boreas-....`). You can create a symlink from boreas dataset on /nas to this directory.

```
# define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=${VTRROOT}/vtr_testing_radar # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/boreas    # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}
```

4. Define path to test scripts

```
ODOMETRY_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry.sh"
ODOMETRY_EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry_eval.sh"
```

These are bash scripts that will run odometry test (using `ros2 run ...`) and evaluation.

5. Run odometry tests in parallel

The following code runs at most `GROUPSIZE` odometry tests in parallel by calling the `$ODOMETRY_SCRIPT` test script with each of the sequence specified in `SEQUENCES`.

```
declare -A pids
for seq in ${SEQUENCES[@]}; do
  echo "Executing command: bash $ODOMETRY_SCRIPT $seq &>/dev/null &"
  ### command to execute
  bash $ODOMETRY_SCRIPT $seq &>/dev/null &
  ###
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
```

6. Run evaluation

When all sequences are finished, the following code runs pyboreas odometry evaluation on the result of each sequence. You should see output in terminal.

```
for seq in ${SEQUENCES[@]}; do
  echo "Executing command: bash $ODOMETRY_EVAL_SCRIPT $seq"
  bash $ODOMETRY_EVAL_SCRIPT $seq
done
```

## [License](./LICENSE)
