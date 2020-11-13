#! /usr/bin/env bash
export UDISTRO=$(lsb_release -sc)
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 373A37D40E480F96524A4027CADA0F77901522B3
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sed $(pwd)/.github/workflows/confs/pbuilderrc.conf.in -e "s|@distro@|${UDISTRO}|g" > $HOME/.pbuilderrc
sudo ln -s $HOME/.pbuilderrc /root/.pbuilderrc
sudo apt update
sudo apt install pbuilder devscripts debootstrap debhelper dh-make -y
sudo pbuilder create
echo "apt install -y lsb-release ubuntu-keyring curl gnupg2 debhelper cmake" \
	| sudo pbuilder --login --save-after-exec
