# radar_topometric_localization

Paper: [Are We Ready for Radar to Replace Lidar in All-Weather Mapping and Localization?](https://ieeexplore.ieee.org/abstract/document/9835037)

Dataset: [Boreas](https://www.boreas.utias.utoronto.ca/#/)

## Installation

Clone this repository and its submodules (recursively).

We use docker to install dependencies The recommended way to build the docker image is

```bash
docker build -t radar_loc \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} .
```

When starting a container, remember to mount the code and datasets directories to proper locations in the container. An example command to start a docker container with the image is

```bash
docker run -it --name radar_loc \
  --privileged \
  --network=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}:${HOME}:rw \
  radar_loc
```

(Inside Container) Go to the root directory of this repository and build STEAM-ICP

```bash
bash build.sh
```

## Experiments

TODO

## Evaluation

TODO

## Citation

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

## [License](./LICENSE)
