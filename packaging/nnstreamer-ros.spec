Name:       nnstreamer-ros
Summary:    nnstreamer extension plugins for ROS support
Version:    0.0.1
Release:    0
Group:      Applications/Multimedia
License:    LGPL-2.1
Source0:    nnstreamer-ros-%{version}.tar.gz
Source1001: nnstreamer-ros.manifest

BuildRequires:  cmake
# boost
BuildRequires:  pkg-config
BuildRequires:  pkgconfig(boost)
# gstremaer
BuildRequires:  pkgconfig(gstreamer-1.0)
BuildRequires:  pkgconfig(gstreamer-base-1.0)
BuildRequires:  pkgconfig(gstreamer-audio-1.0)
BuildRequires:  pkgconfig(gstreamer-video-1.0)
# nnstreamer
BuildRequires:  pkgconfig(nnstreamer)
# ROS
BuildRequires:  ros-kinetic-catkin
BuildRequires:  ros-kinetic-genmsg
BuildRequires:  ros-kinetic-message-generation
BuildRequires:  ros-kinetic-roscpp
# tizen
BuildRequires:  pkgconfig(dlog)

%description
A set of NNStreamer extension plugins for ROS support

%prep
%setup -q
cp %{SOURCE1001} .

%build
%{__ros_setup}
%__ros_build_pkg '-DTIZEN=ON' '-DNNS_INSTALL_LIBDIR=%{_libdir}' '-DENABLE_TEST=OFF'

%install
%{__ros_setup}
%{__ros_install}

%files
%defattr(-,root,root,-)
%manifest nnstreamer-ros.manifest
%license LICENSE
%{_libdir}/gstreamer-1.0/libtensor_ros_sink.so
%{__ros_install_path}/lib/libnns_ros_bridge.so
%{__ros_install_path}/lib/python2.7/site-packages/nns_ros_bridge/*
%{__ros_install_path}/share/nns_ros_bridge/cmake/*
%{__ros_install_path}/share/nns_ros_bridge/msg/*
%{__ros_install_path}/share/nns_ros_bridge/package.xml
%{__ros_install_path}/include/nns_ros_bridge/tensors.h
%{__ros_install_path}/lib/pkgconfig/nns_ros_bridge.pc
# pyc
%exclude %{__ros_install_path}/lib/python2.7/site-packages/nns_ros_bridge/*.pyc
%exclude %{__ros_install_path}/lib/python2.7/site-packages/nns_ros_bridge/msg/*.pyc
# nodejs
%exclude %{__ros_install_path}/share/gennodejs/ros/nns_ros_bridge/msg/*.js
%exclude %{__ros_install_path}/share/gennodejs/ros/nns_ros_bridge/*.js
# lisp
%exclude %{__ros_install_path}/share/roseus/ros/nns_ros_bridge/*
%exclude %{__ros_install_path}/share/common-lisp/ros/nns_ros_bridge/msg/*

