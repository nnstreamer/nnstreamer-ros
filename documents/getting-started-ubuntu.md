# Getting Started: Ubuntu

## Prerequisites
#### Common: Installing the NNStreamer Development Kit package via APT
The NNStreamer packages for Ubuntu are released at a [PPA](https://launchpad.net/~nnstreamer/+archive/ubuntu/ppa) repository. In order to add this APT repository to your system,

```bash
$ sudo add-apt-repository ppa:nnstreamer/ppa
$ sudo apt update
$ sudo apt install nnstreamer-dev
```

- [NNStreamer's PPA](https://launchpad.net/~nnstreamer/+archive/ubuntu/ppa) does not provide the packages for Ubutnu 20.04.

#### Installing the base packages for ROS2

Currently, the Ubuntu packages for Eloquent Elusor are provided only for 18.04 by [the official APT repository](http://packages.ros.org/ros2/ubuntu).

```bash
$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
$ sudo apt update
```

Then, you can install the ROS2 Desktop or Base virtual packages as follows:
```bash
$ sudo apt install ros-eloquent-ros-base
```
or
```bash
$ sudo apt install ros-eloquent-ros-desktop
```

#### Installing the base packages for ROS
The ROS distros namely Kinetic Kame, Melodic Morenia, and Noetic Ninjemys are supported by Ubuntu 16.04, 18.04, and 20.04, respectively via [the official APT repository](http://packages.ros.org/ros/ubuntu) for ROS.

```bash
$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt update
```

Then, you can install the ROS Desktop or Base virtual packages as follows:

```bash
$ sudo apt install ros-${ROSDIST}-ros-base
```
or
```bash
$ sudo apt install ros-${ROSDIST}-ros-desktop
```

Where the ```ROSDIST``` is an environment variable indicating the code name of the ROS distros, kinetic (16.04), melodic (18.04), and noetic (20.04).


## Installing binary packages using APT
The Ubuntu packages for ```nnstreamer-ros2``` and ```nnstreamer-ros``` are released at [this repository](https://bintray.com/beta/#/nnsuite/nnstreamer-ros?tab=packages) powered by JFrog Bintray. To add this repository into your system,
```bash
$ echo "deb [trusted=yes] https://dl.bintray.com/nnsuite/nnstreamer-ros  $(lsb_release -cs) main" | sudo tee -a /etc/apt/sources.list.d/nns-ros.list
$ sudo apt update
```

```nnstreamer-ros2``` and ```nnstreamer-ros``` are the names of packages themselves so that just run ```apt install``` with these names to install NNStreamer-ROS.
```bash
$ sudo apt install nnstreamer-ros2
```
or
```bash
$ sudo apt install nnstreamer-ros
```

## Building from the source code
#### Cloning the source code
```bash
$ git clone https://github.com/nnstreamer/nnstreamer-ros.git
$ cd nnstreamer-ros
```

#### Configuring via CMake
```bash
$ source /opt/ros/${ROSDIST}/setup.sh
$ mkdir -p build
$ cd build
$ cmake .. -DROS_VERSION=${ROSDIST}
```

Where the ```ROSDIST``` is an environment variable indicating the code name of the ROS2 or ROS distros, eloquent (18.04), kinetic (16.04), melodic (18.04), and noetic (20.04).

#### Building using Make
```bash
$ make -j$(nproc)
```

#### Installation (optional)
```bash
$ make install
```
