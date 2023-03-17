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

# Introduction
This repository is a wrapper for two development repositories, `vtr3` and `vtr_testing_radar`, as well as an evaluation repository `pyboreas`. The goal of this wrapper is to make it easy to reproduce the results presented in the paper cited above. All involved repositories are hosted as submodules in the `external` folder, with specific hashes that have been verified to collaborate together. The installation and experiment sections below provide specific commands to get the wrapper up and running in order to be able to re-generate the results in the paper. The instructions make heavy use of helper scripts, that automate and simplify most of the process. The scripts are commented to hopefully make it easier for a user to modify or expand them for their own applications.

# Installation

## Installation Setup
First, clone this repository and recursively initialize submodules using

```Bash 
git clone git@github.com:utiasASRL/radar_topometric_localization.git
git submodule update --init --recursive
```

Then, enter the radar_topometric_localization directory and set it as the root directory using
```Bash
export ROOTDIR=$(pwd)
```

## Setup Docker container
Install nvidia docker runtime first: https://nvidia.github.io/nvidia-container-runtime/

Now, build the docker image using
```Bash
cd $ROOTDIR
source setup_scripts/build_docker.sh
```

After the image is built (this will take a while), launch a container using
```Bash
source setup_scripts/run_docker.sh
```
Make sure the `${ROOTDIR}` variable is initialized! Note, this script will launch a new container if none currently exist and will join an existing container if one was already launched. FYI: to start a new terminal with the existing container: `docker exec -it radar_loc bash`

Next, **inside the contaner**, build the vtr3 and vtr_testing_radar packages using (this will take a while the first time)
```Bash
source setup_scripts/build_packages.sh
```

Finally, **inside the contaner**, install the pyboreas evaluation tools using
```Bash
source setup_scripts/create_venv.sh
```

# Running Experiments
All commands in this section must be run **inside the contaner**!

Set up all required variables by running
```Bash
source setup_scripts/setup_container.sh
```
Make sure all variables point to directories in your specific setup!

## Visualization (Work in Progress)

### RVIZ
First launch RVIZ for visualization:

```Bash
source /opt/ros/humble/setup.bash               # source the ROS environment
ros2 run rviz2 rviz2 -d $ROOTDIR/external/vtr3/rviz/radar.rviz # launch rviz
```

Then in another terminal, launch `rqt_reconfigure` for control. Currently supported dynamic reconfigure parameters: `control_test.play` and `control_test.delay_millisec`

```Bash
source /opt/ros/humble/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
```

### Foxglove
An alternative visualization approach is to use [Foxglove](https://foxglove.dev). This approach has the advantage of being able to locally visualize ROS topics even in cases where the code is running on a remote machine. For convinience, the Foxglove WebSocket is already installed as part of the standard Dockerfile. This allows you to connect to the remote machine using the web browser or by downloading the [Foxglove Studio](https://foxglove.dev/download), as long as your local machine can reach the remote machine in some manner. 

To use the WebSocket, open another terminal window inside of a set up Docker container and run
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then, either [in the browser](https://studio.foxglove.dev) or in the Foxglove Studio application, navigate to `Open Connection -> Foxglove WebSocket` and enter `ws://REMOTE_IP:8765`, where `REMOTE_IP` is the ping-able IP address of your remote machine. Afterwards, all remote machine ROS topics should be visualizable using the Foxglove interface (once a test is running). Additional information about using Foxglove WebSocket can be found at https://github.com/foxglove/ros-foxglove-bridge/.

## Odometry (Teach) and Localization (Repeat)

Make sure that `setup_container.sh` has been run, as it defines all required variables and sources all required packages! Additionally, launch the following commands from the root directory.

The general form of running and evaluating a test is
```Bash
bash runtime/run_test.sh ${MODE} ${SENSOR} ${SEQUENCES}
bash runtime/run_eval.sh ${MODE} ${SENSOR} ${SEQUENCES}
```
where `MODE = [odometry, localization]`, `SENSOR = [radar, lidar, radar_lidar]`, and `SEQUENCES` is either one (for odometry) or two (for localization) Boreas sequence names. Consider the examples below for radar.

Consider, as an example, the following sequences for odometry and localization.
```Bash
# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2020-11-26-13-58
LOC_INPUT=boreas-2021-01-26-10-59
```
Note, it is not required to define these variables, as you can input the sequence name as an argument directly. If it is desired to do a localization test, it is first required that an odometry result is generated for the sequence against which a localization attempt is desired.

Run and evaluate a single radar odometry test using
```Bash
bash runtime/run_test.sh odometry radar ${ODO_INPUT}
bash runtime/run_eval.sh odometry radar ${ODO_INPUT}
```

Run and evaluate a single radar localization test using
```Bash
bash runtime/run_test.sh localization radar ${ODO_INPUT} ${LOC_INPUT}
bash runtime/run_eval.sh localization radar ${ODO_INPUT}
```

Note, that the evaluation scripts both only take in an odometry sequence. This is because the output of a localization run against a map constructed from an odometry sequence is stored under the odometry sequence result subfolder. The evaluation script evaluates all localization sequences contained within the odometry sequence subfolder at the same time.

This code has been tested with the sequences found in the paper. There may be difficulties running it with newer boreas sequences, as the sensor configurations may have changed.

## Running Experiments in Parallel

Assuming you want to run odometry or localization for multiple test sequences in parallel, it is possible to do so by running

The general form of running and evaluating tests on multiple sequences in parallel is
```Bash
bash runtime/run_parallel_test.sh ${MODE} ${SENSOR}
```
where `MODE = [odometry, localization]`, `SENSOR = [radar, lidar, radar_lidar]`. This script runs all tests, either odometry or localization, and evaluates them afterwards. Note, SEQUENCES are not provided as an input for this script, as the specific list of sequences desired to be tested in parallel must be set inside of the script file. Consider the examples below for radar localization.

```Bash
bash runtime/run_parallel_test.sh localization radar
```
Note, running this script assumes that the REFERENCE sequence, set inside of run_parallel_test.sh, has an already completed odometry test.

You can monitor the progress of each test by going to the log file of each test.

The log file should be located at

`${VTRRESULT}/${SENSOR}/${ODO_INPUT}/${ODO_INPUT}/<some name based on time>.log`

for odometry and at

`${VTRRESULT}/${SENSOR}/${ODO_INPUT}/${LOC_INPUT}/<some name based on time>.log`

for localization, where `${VTRRESULT}` is set in `setup_container.sh`. After the evaluation of the tests is complete, you should see the output in the terminal. Various other results can be found in the `${VTRRESULT}` directory.

## Installing and Running a Test Sequence
This repository is set up to run with Boreas seqences, which can be installed from `https://www.boreas.utias.utoronto.ca/#/download`. To facilitate downloading the sequences, the Docker image is set up with the necessary AWS CLI. A script to install a test sequence (approx. 5 Gb) is included to facilitate verifying that all installation has completed sucessfully. This script will install the test sequence in the `data` folder. To download the sequence, run

```Bash
bash setup_scripts/dl_boreas_test.sh
```

After the sequence has downloaded, this may take a while depending on your download speed, you can run a radar odometry test on the sequence through

```Bash
bash runtime/run_test.sh odometry radar boreas-2020-11-26-13-58
```

The expected output, assuming nothing has been changed from the default configuration file, should be similar to the following

```shell
WARNING [boreas_odometry.cpp:149] [test] Found 4142 radar data
WARNING [boreas_odometry.cpp:169] [test] Loading radar frame 0 with timestamp 1606417097528152000
WARNING [boreas_odometry.cpp:169] [test] Loading radar frame 1 with timestamp 1606417097778155000
WARNING [odometry_icp_module.cpp:553] [radar.odometry_icp] T_m_r is:     0.007637     0.015923 -1.07134e-05  2.18899e-06  8.45092e-08   0.00219319
WARNING [odometry_icp_module.cpp:554] [radar.odometry_icp] w_m_r_in_r is:   -0.0214213   -0.0636917  4.28452e-05 -7.50626e-06  1.60892e-07  -0.00437644
WARNING [boreas_odometry.cpp:169] [test] Loading radar frame 2 with timestamp 1606417098028164000
WARNING [odometry_icp_module.cpp:553] [radar.odometry_icp] T_m_r is:    0.0194236    0.0177124  0.000548001 -4.29419e-05 -1.01629e-05   0.00356502
WARNING [odometry_icp_module.cpp:554] [radar.odometry_icp] w_m_r_in_r is:  0.00215017 -0.00707829 -0.00223349 0.000172793 3.52718e-05   0.0107883
```

Consult the [Boreas download page](https://www.boreas.utias.utoronto.ca/#/download) and the example download script to download additional sequences. Remember that localization can only be run once an odometry test has been run on a different sequence.


## [License](./LICENSE)