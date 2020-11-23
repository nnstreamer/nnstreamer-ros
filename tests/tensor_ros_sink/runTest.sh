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
rm -rf *.bag
rm -rf *.log

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

TENSOR_ROS_SINK="tensor_ros_sink"
if [[ "$ROS_VERSION" == "2" ]]
then
	TENSOR_ROS_SINK="tensor_ros2_sink"
fi

OUTPUT=simple.BGRx.uint8.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=BGRx,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.BGRx.uint8.bag\" sync=true t. ! queue ! filesink location=\"simple.BGRx.uint8.log\" sync=true" 1-1
../test_utils.py simple.BGRx.uint8.log ${OUTPUT} BGRx uint8 280 40 ${ROS_VERSION}
testResult $? 1-1 "tensor_ros_sink: simple test case using raw data (video/BGRx/uint8)" 0 1

OUTPUT=simple.RGB.uint8.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi

gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=RGB,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.RGB.uint8.bag\" sync=true t. ! queue ! filesink location=\"simple.RGB.uint8.log\" sync=true" 1-2
../test_utils.py simple.RGB.uint8.log ${OUTPUT} RGB uint8 280 40 ${ROS_VERSION}
testResult $? 1-2 "tensor_ros_sink: simple test case using raw data (video/RGB/uint8)" 0 1

OUTPUT=simple.GRAY8.uint8.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} videotestsrc num-buffers=1 ! video/x-raw,format=GRAY8,width=280,height=40,framerate=0/1 ! tee name=t ! queue ! tensor_converter silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.GRAY8.uint8.bag\" sync=true t. ! queue ! filesink location=\"simple.GRAY8.uint8.log\" sync=true" 1-3
../test_utils.py simple.GRAY8.uint8.log ${OUTPUT} GRAY8 uint8 280 40 ${ROS_VERSION}
testResult $? 1-3 "tensor_ros_sink: simple test case using raw data (video/GRAY8/uint8)" 0 1

OUTPUT=simple.audio.1.s16le.8k.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi
# audio format mono, S16LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=S16LE,rate=8000,channels=1 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.audio.1.s16le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.1.s16le.8k.log\" sync=true" 2-1
../test_utils.py simple.audio.1.s16le.8k.log ${OUTPUT} 1 s16le 8000 ${ROS_VERSION}
testResult $? 2-1 "tensor_ros_sink: simple test case using raw data (audio/mono/s16le/8k)" 0 1

OUTPUT=simple.audio.2.s16le.8k.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi
# audio format stereo, S16LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=S16LE,rate=8000,channels=2 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.audio.2.s16le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.2.s16le.8k.log\" sync=true" 2-2
../test_utils.py simple.audio.2.s16le.8k.log ${OUTPUT} 2 s16le 8000 ${ROS_VERSION}
testResult $? 2-2 "tensor_ros_sink: simple test case using raw data (audio/stereo/s16le/8k)" 0 1

OUTPUT=simple.audio.1.f32le.8k.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi

# audio format mono, F32LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F32LE,rate=8000,channels=1 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.audio.1.f32le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.1.f32le.8k.log\" sync=true" 2-3
../test_utils.py simple.audio.1.f32le.8k.log ${OUTPUT} 1 f32le 8000 ${ROS_VERSION}
testResult $? 2-3 "tensor_ros_sink: simple test case using raw data (audio/mono/f32le/8k)" 0 1

OUTPUT=simple.audio.2.f32le.8k.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi
# audio format stereo, F32LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F32LE,rate=8000,channels=2 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.audio.2.f32le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.2.f32le.8k.log\" sync=true" 2-4
../test_utils.py simple.audio.2.f32le.8k.log ${OUTPUT} 2 f32le 8000 ${ROS_VERSION}
testResult $? 2-4 "tensor_ros_sink: simple test case using raw data (audio/stereo/f32le/8k)" 0 1

OUTPUT=simple.audio.1.f64le.8k.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi
# audio format mono, F64LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F64LE,rate=8000,channels=1 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.audio.1.f64le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.1.f64le.8k.log\" sync=true" 2-5
../test_utils.py simple.audio.1.f64le.8k.log ${OUTPUT} 1 f64le 8000 ${ROS_VERSION}
testResult $? 2-5 "tensor_ros_sink: simple test case using raw data (audio/mono/f64le/8k)" 0 1

OUTPUT=simple.audio.2.f64le.8k.bag
if [[ "$ROS_VERSION" == "2" ]]
then
	OUTPUT=${OUTPUT}/${OUTPUT}_0.db3
fi
# audio format stereo, F64LE, 8k sample rate, samples per buffer 8000
gstTest "-m -v --gst-plugin-path=${PATH_TO_PLUGIN} audiotestsrc num-buffers=1 samplesperbuffer=8000 ! audioconvert ! audio/x-raw,format=F64LE,rate=8000,channels=2 ! tee name=t ! queue ! tensor_converter frames-per-tensor=8000 silent=TRUE ! ${TENSOR_ROS_SINK} dummy=true enable-save-rosbag=true location=\"simple.audio.2.f64le.8k.bag\" sync=true t. ! queue ! filesink location=\"simple.audio.2.f64le.8k.log\" sync=true" 2-6
../test_utils.py simple.audio.2.f64le.8k.log ${OUTPUT} 2 f64le 8000 ${ROS_VERSION}
testResult $? 2-6 "tensor_ros_sink: simple test case using raw data (audio/stereo/f64le/8k)" 0 1

report
