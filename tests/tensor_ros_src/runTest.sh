#!/usr/bin/env bash
##
## @file runTest.sh
## @author Wook Song <wook16.song@samsung.com>
## @date  Nov 25 2029
## @brief SSAT test cases for NNStreamer-ros
#
if [[ "$SSATAPILOADED" != "1" ]]
then
	SILENT=0
	INDEPENDENT=1
	search="ssat-api.sh"
	source $search
	printf "${Blue}Independent Mode${NC}"
fi
testInit "rosbag-based pipeline test"
rm -rf *.bag *.log *.db3-shm *.db3-wal

if [[ -z ${BUILD_PATH} ]]
then
	BUILD_PATH="../../build"
fi

if [[ ! -d ${BUILD_PATH} ]]
then
	printf "\"%s\" is not valid BUILD_PATH\n" ${BUILD_PATH}
	exit 1
fi

PATH_TO_PLUGIN=${BUILD_PATH}/gst/tensor_ros_sink:${BUILD_PATH}/gst/tensor_ros_src

TENSOR_ROS_SRC="tensor_ros_src"
GOLDEN=ros.bag.golden
if [[ "$ROS_VERSION" == "2" ]]
then
	TENSOR_ROS_SRC="tensor_ros2_src"
	GOLDEN=ros2.bag.golden.db3
fi

gst-launch-1.0 filesrc location=orange.png ! pngdec ! videoscale ! imagefreeze ! videoconvert ! video/x-raw,format=RGB,framerate=0/1 ! tensor_converter ! filesink location=\"orange.RGB.tensors.golden.log\"

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} ${TENSOR_ROS_SRC} enable-load-rosbag=true location=${GOLDEN} ! tee name=t ! queue ! filesink location=\"orange.RGB.tensors.log\"" 1-1
diff orange.RGB.tensors.log orange.RGB.tensors.golden.log
testResult $? 1-1 "tensor_ros_sink: simple test case using a png file (video/RGB)" 0 1

report
