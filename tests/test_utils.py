#!/usr/bin/env python

##
# Copyright (C) 2019 Samsung Electronics
# License: LGPL-2.1
#
# @file test_utils.py
# @brief Utility functions for test cases
# @author Wook Song <wook16.song@samsung.com>

import sys
import os
import struct
import rosbag

##
# @brief Get the number of colors in a pixel from a given 'colorspace'
# @param[in] colorspace 'RGB','BGR', 'RGBx', 'BGRx', 'xRGB', 'xBGR', 'RGBA', 'BGRA', 'ARGB', 'ABGR', 'GRAY8'
# @return The number of colors or -1 if it fails


def get_colors_per_px(colorspace):
    if colorspace in ['RGB', 'BGR']:
        colors_per_px = 3
    elif colorspace in ['RGBx', 'BGRx', 'xRGB', 'xBGR', 'RGBA', 'BGRA', 'ARGB', 'ABGR']:
        colors_per_px = 4
    elif colorspace in ['GRAY8']:
        colors_per_px = 1
    else:
        print('Unsupported colorspace {}'.format(colorspace))
        return -1

    return colors_per_px

##
# @brief Get a format string to pass to struct.unpack and the number of bytes per data
# @param[in] data_type a string indicating the data type such as uint8, uint16, u16le, u16be...
# @param[in] num_data the number of data
# @param[in] byte_order 0 indicates 'little-endian' and the others indicate 'big-endian' (optional)
# @return (string_format, num_bytes_per_data)


def get_unpack_format_and_bytes_per_data(data_type, num_data, byte_order=0):

    if byte_order == 0:
        unpack_format = '<'
    else:
        unpack_format = '>'

    unpack_format += str(num_data)

    lower_data_type = data_type.lower()
    if lower_data_type in ['uint8', 'int8', 's8', 'u8']:
        bytes_per_data = 1
        if lower_data_type in ['uint8', 'u8']:
            unpack_format += 'B'
        else:
            unpack_format += 'b'
    elif lower_data_type in ['uint16', 'int16', 's16le', 's16be', 'u16le', 'u16be']:
        bytes_per_data = 2
        if lower_data_type in ['uint16', 'U16le', 'U16be']:
            unpack_format += 'H'
        else:
            unpack_format += 'h'
    elif lower_data_type in ['uint32', 'int32', 'float32', 's32le', 's32be', 'u32le', 'u32be', 'f32le', 'f32be']:
        bytes_per_data = 4
        if lower_data_type in ['uint32', 'u32le', 'u32be']:
            unpack_format += 'I'
        elif lower_data_type in ['int32', 's32le', 's32be']:
            unpack_format += 'i'
        else:
            unpack_format += 'f'
    elif lower_data_type in ['uint64', 'int64', 'float64', 'f64le', 'f64be']:
        bytes_per_data = 8
        if lower_data_type == 'uint64':
            unpack_format += 'Q'
        elif lower_data_type == 'int32':
            unpack_format += 'q'
        else:
            unpack_format += 'd'
    else:
        print('Unsupported data type {}'.format(data_type))
        return (None, -1)

    return (unpack_format, bytes_per_data)

##
# @brief Get a tuple of data stream in a tensor from a raw data file
# @param[in] data_type a string indicating the data type such as uint8, uint16...
# @param[in] colorspace 'RGB','BGR', 'RGBx', 'BGRx', 'xRGB', 'xBGR', 'RGBA', 'BGRA', 'ARGB', 'ABGR', 'GRAY8'
# @param[in] width the width of the image
# @param[in] height the height of the image
# @param[in] filename a name of file that contains raw data of the image
# @param[in] byte_order 0 indicates 'little-endian' and the others indicate 'big-endian' (optional)
# @return a tuple of data stream in a tensor or None (if it fails)


def get_tensor_stream_from_raw_video(data_type, colorspace, width, height, filename, byte_order=0):
    try:
        file_raw = open(filename, 'rb')
    except IOError:
        print('Failed to open the raw data file: {}'.format(filename))
        return None
    byte_stream = file_raw.read()
    file_raw.close()

    colors_per_px = get_colors_per_px(colorspace)
    if colors_per_px is None:
        return None
    num_data = colors_per_px * width * height
    unpack_format, bytes_per_data = get_unpack_format_and_bytes_per_data(
        data_type, num_data)
    if unpack_format is None and bytes_per_data == -1:
        return None

    size = bytes_per_data * num_data
    if size != os.path.getsize(filename):
        print('Failed to verify the raw data file: {}'.format(filename))
        return None

    data_stream = struct.unpack_from(unpack_format, byte_stream)

    return data_stream

##
# @brief Get a tuple of data stream in a tensor from a rosbag file
# @param[in] filename a name of file that contains raw data of the image
# @param[in] byte_order 0 indicates 'little-endian' and the others indicate 'big-endian' (optional)
# @return a tuple of data stream in a tensor or None (if it fails)


def get_tensor_stream_from_bag(filename, byte_order=0):

    MSG_FORMAT_OPEN_ERR = 'Failed to open the rosbag file: {}: '.format(
        filename)
    IDX_MSG_IN_TUPLE = 1

    try:
        bag = rosbag.Bag(filename)
    except IOError:
        print(MSG_FORMAT_OPEN_ERR + 'the file does not exist')
        return None
    except rosbag.bag.ROSBagException:
        print(MSG_FORMAT_OPEN_ERR + 'the file is not a rosbag file')
        return None
    except rosbag.bag.ROSBagFormatException:
        print(MSG_FORMAT_OPEN_ERR + 'the file has crashed')
        return None

    # Verfication procedure:
    # This function is only for the case that the bag file has a tensor in a single msg
    #   1. check that the number of msg is 1
    #   2. check that the number of tensors is 1
    #   3. check that the number of dimensions in a tensor is 4
    #   4. check that data types of all the dimensions are same
    num_msg = bag.get_message_count()
    if num_msg != 1:
        bag.close()
        return None

    gen_tuple = bag.read_messages()
    msg = next(gen_tuple)[IDX_MSG_IN_TUPLE]
    if len(msg.tensors) != 1:
        bag.close()
        return None

    bag.close()
    if len(msg.tensors[0].layout.dim) != 4:
        return None

    data_type = msg.tensors[0].layout.dim[0].label
    if msg.tensors[0].layout.dim[1].label != data_type or \
       msg.tensors[0].layout.dim[2].label != data_type or \
       msg.tensors[0].layout.dim[3].label != data_type:
        return None
    num_data = msg.tensors[0].layout.dim[3].size * \
        msg.tensors[0].layout.dim[2].size * \
        msg.tensors[0].layout.dim[1].size

    unpack_format, bytes_per_data = get_unpack_format_and_bytes_per_data(
        data_type, num_data)
    if unpack_format is None and bytes_per_data == -1:
        return None

    byte_stream = msg.tensors[0].data
    if len(byte_stream) != num_data * bytes_per_data:
        return None

    data_stream = struct.unpack_from(unpack_format, byte_stream)

    return data_stream


##
# @brief Compare a file containing raw data for a video frame with a rosbag file corresponding to such raw data file
# @param[in] argv a list of command-link arguments


def compare_video_raw_and_bag(argv):
    filename_raw = sys.argv[1]
    filename_rosbag = sys.argv[2]
    colorspace = sys.argv[3]
    data_type = sys.argv[4]
    width = int(sys.argv[5])
    height = int(sys.argv[6])

    data_stream_raw = get_tensor_stream_from_raw_video(
        data_type, colorspace, width, height, filename_raw)
    if data_stream_raw is None or len(data_stream_raw) == 0:
        exit(1)

    data_stream_bag = get_tensor_stream_from_bag(filename_rosbag)
    if data_stream_bag is None or len(data_stream_bag) == 0:
        exit(1)
    if data_stream_raw != data_stream_bag:
        exit(1)

##
# @brief Get a tuple of data stream in a tensor from a raw data file
# @param[in] data_type a string indicating the data type such as S16LE, S16BE, U16LE, U16BE...
# @param[in] channels an integer [1..2147483647] indicating the number of channels
# @param[in] frames_per_tensor an integer indicating the number of frames in a tensor
# @param[in] byte_order 0 indicates 'little-endian' and the others indicate 'big-endian'
# @param[in] filename a name of file that contains raw data of the image
# @return a tuple of data stream in a tensor or None (if it fails)


def get_tensor_stream_from_raw_audio(data_type, channels, frames_per_tensor, byte_order, filename):
    try:
        file_raw = open(filename, 'rb')
    except IOError:
        print('Failed to open the raw data file: {}'.format(filename))
        return None
    byte_stream = file_raw.read()
    file_raw.close()

    num_data = channels * frames_per_tensor
    unpack_format, bytes_per_data = get_unpack_format_and_bytes_per_data(
        data_type, num_data, byte_order)
    if unpack_format is None and bytes_per_data == -1:
        return None

    size = bytes_per_data * num_data
    if size != os.path.getsize(filename):
        print('Failed to verify the raw data file: {}'.format(filename))
        return None
    data_stream = struct.unpack_from(unpack_format, byte_stream)

    return data_stream


##
# @brief Compare a file containing raw data for a set of audio frames with a rosbag file corresponding to such raw data file
# @param[in] argv a list of command-link arguments


def compare_audio_raw_and_bag(argv):
    filename_raw = sys.argv[1]
    filename_rosbag = sys.argv[2]
    channels = int(sys.argv[3])
    data_type = sys.argv[4]
    frames_per_tensor = int(sys.argv[5])

    byte_order = data_type.find('BE')
    if byte_order == -1:
        byte_order = 0

    data_stream_raw = get_tensor_stream_from_raw_audio(
        data_type, channels, frames_per_tensor, byte_order, filename_raw)
    if data_stream_raw is None or len(data_stream_raw) == 0:
        exit(1)

    data_stream_bag = get_tensor_stream_from_bag(filename_rosbag, byte_order)
    if data_stream_bag is None or len(data_stream_bag) == 0:
        exit(1)
    if data_stream_raw != data_stream_bag:
        exit(1)


print(sys.argv[3])
if sys.argv[3] in ['RGB','BGR', 'RGBx', 'BGRx', 'xRGB', 'xBGR', \
    'RGBA', 'BGRA', 'ARGB', 'ABGR', 'GRAY8']:
    if len(sys.argv) != 7:
        exit(1)
    compare_video_raw_and_bag(sys.argv)
elif int(sys.argv[3]) > 0 and int(sys.argv[3]) <= 2147483647:
    if len(sys.argv) != 6:
        exit(1)
    compare_audio_raw_and_bag(sys.argv)
else:
    exit(1)

exit(0)
