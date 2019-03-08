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

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=BGRx,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.BGRx.uint8.bag\" sync=true t. ! queue ! filesink location=\"simple.BGRx.uint8.log\" sync=true" 1-1
../test_utils.py simple.BGRx.uint8.log simple.BGRx.uint8.bag BGRx uint8 280 40
testResult $? 1-1 "tensor_ros_sink: simple test case using raw data (video/BGRx/uint8)" 0 1

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=RGB,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.RGB.uint8.bag\" sync=true t. ! queue ! filesink location=\"simple.RGB.uint8.log\" sync=true" 1-2
../test_utils.py simple.RGB.uint8.log simple.RGB.uint8.bag RGB uint8 280 40
testResult $? 1-2 "tensor_ros_sink: simple test case using raw data (video/RGB/uint8)" 0 1

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=GRAY8,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.GRAY8.uint8.bag\" sync=true t. ! queue ! filesink location=\"simple.GRAY8.uint8.log\" sync=true" 1-3
../test_utils.py simple.GRAY8.uint8.log simple.GRAY8.uint8.bag GRAY8 uint8 280 40
testResult $? 1-3 "tensor_ros_sink: simple test case using raw data (video/GRAY8/uint8)" 0 1

# audio format mono, S16LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=S16LE,rate=8000,channels=1 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.audio.1.s16le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.1.s16le.8k.log\" sync=true" 2-1
../test_utils.py simple.audio.1.s16le.8k.log simple.audio.1.s16le.8k.bag 1 s16le 8000
testResult $? 2-1 "tensor_ros_sink: simple test case using raw data (audio/mono/s16le/8k)" 0 1

# audio format stereo, S16LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=S16LE,rate=8000,channels=2 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.audio.2.s16le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.2.s16le.8k.log\" sync=true" 2-2
../test_utils.py simple.audio.2.s16le.8k.log simple.audio.2.s16le.8k.bag 2 s16le 8000
testResult $? 2-2 "tensor_ros_sink: simple test case using raw data (audio/stereo/s16le/8k)" 0 1

# audio format mono, F32LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F32LE,rate=8000,channels=1 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.audio.1.f32le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.1.f32le.8k.log\" sync=true" 2-3
../test_utils.py simple.audio.1.f32le.8k.log simple.audio.1.f32le.8k.bag 1 f32le 8000
testResult $? 2-3 "tensor_ros_sink: simple test case using raw data (audio/mono/f32le/8k)" 0 1

# audio format stereo, F32LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F32LE,rate=8000,channels=2 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.audio.2.f32le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.2.f32le.8k.log\" sync=true" 2-4
../test_utils.py simple.audio.2.f32le.8k.log simple.audio.2.f32le.8k.bag 2 f32le 8000
testResult $? 2-4 "tensor_ros_sink: simple test case using raw data (audio/stereo/f32le/8k)" 0 1

# audio format mono, F64LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F64LE,rate=8000,channels=1 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.audio.1.f64le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.1.f64le.8k.log\" sync=true" 2-5
../test_utils.py simple.audio.1.f64le.8k.log simple.audio.1.f64le.8k.bag 1 f64le 8000
testResult $? 2-5 "tensor_ros_sink: simple test case using raw data (audio/mono/f64le/8k)" 0 1

# audio format stereo, F64LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F64LE,rate=8000,channels=2 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! tensor_ros_sink dummy=true enable-save-rosbag=true location=\"simple.audio.2.f64le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.2.f64le.8k.log\" sync=true" 2-6
../test_utils.py simple.audio.2.f64le.8k.log simple.audio.2.f64le.8k.bag 2 f64le 8000
testResult $? 2-6 "tensor_ros_sink: simple test case using raw data (audio/stereo/f64le/8k)" 0 1


report

