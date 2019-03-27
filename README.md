
[![GitHub license](https://img.shields.io/github/license/nnsuite/nnstreamer-ros.svg?style=plastic)](./LICENSE)

# NNStreamer-ROS

[![Code Coverage](http://nnsuite.mooo.com/nnstreamer-ros/ci/badge/codecoverage.svg)](http://nnsuite.mooo.com/nnstreamer-ros/ci/gcov_html/index.html)

## Official Releases

| Arch | [Tizen](http://download.tizen.org/live/devel%3A/AIC%3A/Tizen%3A/5.0%3A/nnsuite/standard/) | [Ubuntu](https://launchpad.net/~nnstreamer/+archive/ubuntu/ppa) | Yocto |
|  :--:  |    :--:    |     :--:    |   :--:  |
|        |    5.5     | 16.04/18.04 |   TBD   |
|   x64  | Available  |   Planned   |   WiP   |
|   x86  | Available  |   Planned   |   WiP   |
|  arm64 | Available  |   Planned   |   WiP   |
|   arm  | Available  |   Planned   |   WiP   |

## Getting Started (WiP)

#### Installing from source
```bash
$ git clone https://github.com/nnsuite/nnstreamer-ros.git
$ cd nnstreamer-ros
$ mkdir -p build
$ cd build
$ cmake ..
$ make
# By default for Ubuntu 16.04 LTS on x64, plugins for the nnstreamer will be installed
# into under /usr/lib/x86_64-linux-gnu/ and ROS related files (shared library, header files
# for custom defined message, pkg-config files, ...) will be installed into under /opt/ros/kinetic/.
$ sudo make install
```
You can change installation directories using cmake custom defined variables such as ROS_VERSION, ROS_BASE_PREFIX, ROS_INSTALL_PREFIX, NNS_INSTALL_PREFIX. For example, ``` cmake .. ``` is equivalent to
```bash
$ cmake .. -DROS_VERSION=kinetic -DROS_BASE_PREFIX=/etc/ros -DROS_INSTALL_PREFIX=/etc/ros \
-DNNS_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=lib/x86_64-linux-gnu \
-DCMAKE_LIBRARY_ARCHITECTURE=x86_64-linux-gnu
```

## Usage Examples (TBD)
