#!/usr/bin/env bash
##
## @file runTest.sh
## @author Wook Song <wook16.song@gmail.com>
## @date  2월 28 2019
## @brief SSAT test cases for NNStreamer-ros
#
if [[ "$SSATAPILOADED" != "1" ]]
then
	SILENT=0
	INDEPENDENT=1
	search="ssat-api.sh"
	source $search
	printf "${Blue}Independent Mode${NC}
"
fi
testInit "rosbag-based pipeline test"

PATH_TO_PLUGIN="../../build"

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=BGRx,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.uint8.BGRx.bag\" sync=true t. ! queue ! filesink location=\"simple.uint8.BGRx.log\" sync=true" 1-1

../test_utils.py simple.uint8.BGRx.log simple.uint8.BGRx.bag uint8 BGRx 280 40
testResult $? 1-1 "tensor_ros_sink: simple test case for raw data (uint8/BGRx)" 0 1

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=RGB,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.uint8.RGB.bag\" sync=true t. ! queue ! filesink location=\"simple.uint8.RGB.log\" sync=true" 1-2
../test_utils.py simple.uint8.RGB.log simple.uint8.RGB.bag uint8 RGB 280 40
testResult $? 1-2 "tensor_ros_sink: simple test case for raw data (uint8/RGB)" 0 1

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=GRAY8,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.uint8.GRAY8.bag\" sync=true t. ! queue ! filesink location=\"simple.uint8.GRAY8.log\" sync=true" 1-3
../test_utils.py simple.uint8.GRAY8.log simple.uint8.GRAY8.bag uint8 GRAY8 280 40
testResult $? 1-3 "tensor_ros_sink: simple test case for raw data (uint8/GRAY8)" 0 1

report

