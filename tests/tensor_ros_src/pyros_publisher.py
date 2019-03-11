#!/usr/bin/env python

import rospy
import std_msgs.msg
import argparse

datatype = {'int32' : std_msgs.msg.Int32MultiArray,
        'uint32' : std_msgs.msg.UInt32MultiArray,
        'int8' : std_msgs.msg.Int8MultiArray,
        'uint8' : std_msgs.msg.UInt8MultiArray,
        'int16' : std_msgs.msg.Int16MultiArray,
        'uint16' : std_msgs.msg.UInt16MultiArray,
        'int64' : std_msgs.msg.Int64MultiArray,
        'uint64' : std_msgs.msg.UInt64MultiArray,
        'float32' : std_msgs.msg.Float32MultiArray,
        'float64' : std_msgs.msg.Float64MultiArray}

def get_stride (l, idx):
    ret = 1
    for i in l[idx:]:
        ret = ret * int(i)
    return ret


class RosNode:
    def __init__(self, node_name, topic_name, datatype, pub_rate, dim_str):
        self.node_name = node_name
        self.topic_name = topic_name
        self.datatype = datatype
        self.pub_rate = pub_rate
        self.dim_str = dim_str

    def print_info(self):
        print ("node_name: %s" % self.node_name)
        print ("topic_name: %s" % self.topic_name)
        print ("datatype: %s" % self.datatype)
        print ("pub_rate: %s" % self.pub_rate)

    def generate_msg (self):
        msg = datatype.get (self.datatype)()

        for i in range(4):
            msg.layout.dim.append (std_msgs.msg.MultiArrayDimension())
            msg.layout.dim[i].size = 1
            msg.layout.dim[i].stride = 1

        dims = self.dim_str.split(':')

        for i in range(len(dims)):
            msg.layout.dim[i].size = int((dims[i]))
            msg.layout.dim[i].stride = get_stride (dims, i)

        msg.layout.data_offset = 0
        max_val = get_stride (dims, 0)

        data = []
        for i in range(max_val):
            data.append(i * 10);

        msg.data = data
        return msg

    def publish (self):
        rospy.init_node (self.node_name, anonymous=True)
        pub = rospy.Publisher (self.topic_name, datatype.get(n.datatype), queue_size = 10)
        rate = rospy.Rate (self.pub_rate)

        while not rospy.is_shutdown ():
            msg = self.generate_msg ()
            rospy.loginfo (msg)
            pub.publish (msg)
            rate.sleep()


def make_param ():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--node_name',
            type=str, help='node name', required=True)
    parser.add_argument('-t', '--topic_name',
            type=str, help='topic name to publish', required=True)
    parser.add_argument('-d', '--data_type',
            type=str, help='primitive datatype for publishing (Default: int32)', default="int32")
    parser.add_argument('-e', '--dimension_string',
            type=str, help='dimension string (e.g. 10:1:1:1)', default="10:1:1:1")
    parser.add_argument('-r', '--pub_rate',
            type=int, help='publishing rate (Hz) (Default: 1Hz)', default=1)

    args = parser.parse_args()
    ret = RosNode (args.node_name, args.topic_name, 
            args.data_type, args.pub_rate, args.dimension_string)

    return ret


if __name__ == '__main__':

    n = make_param ()
    n.print_info ()
    n.publish ()

