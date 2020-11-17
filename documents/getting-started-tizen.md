# Getting Started: Tizen

## Prerequisites
- The nnstreamer-ros packages for Tizen 6.0 can be found [here](http://download.tizen.org/live/devel%3A/Tizen%3A/6.0%3A/AI/Tizen_Unified_standard/).

## Installing GBS

```bash
$ sudo sh -c 'echo "deb [trusted=yes] http://download.tizen.org/tools/latest-release/Ubuntu_$(lsb_release -rs)/ /"> /etc/apt/sources.list.d/tizen-devtools.list'
$ sudo apt update
$ sudo apt install gbs
```

- To build using [GBS](https://docs.tizen.org/platform/reference/gbs/gbs-overview/), an extra repository section should be added to your GBS configuration file as follows.

```bash
[repo.nnstreamer-ros]
url = http://download.tizen.org/live/devel:/Tizen:/6.0:/AI/Tizen_Unified_standard/
```
- In addition, the repository name (i.e., 'repo.nnstreamer-ros' in this example) should be also added to the 'repos' property of the your profile section.

## Cloning the source code
```bash
$ git clone https://github.com/nnstreamer/nnstreamer-ros.git
$ cd nnstreamer-ros
```

## Building from the source code using GBS
```bash
$ gbs build -A aarch64
```
```aarch64``` can be replaced with the other architecture names, which is supported by Tizen, such as ```x86_64```, ```i586```, and ```armv7l```.
