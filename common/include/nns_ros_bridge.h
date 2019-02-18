/**
 * Copyright (C) 2018 Samsung Electronics Co., Ltd. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 */
/**
 * @file   nns_ros_bridge.h
 * @author Wook Song <wook16.song@samsung.com>
 * @date   11/19/2018
 * @brief  A bridge for ROS support within NNStreamer
 *
 * This class bridges between the NNStreamer (C) and ROS frameworks (ROSCPP/C++).
 *
 * @bug     No known bugs.
 */

#ifndef _NNS_ROS_BRIDGE_H_
#define _NNS_ROS_BRIDGE_H_
#include <glib-2.0/glib.h>
#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>
#ifdef __cplusplus
#include <ros/ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

typedef enum _err_code {
  UNDEFINED_ROS_MASTER_URI,
  FAILED_TO_CONNECT_ROSCORE,
} err_code;

class NnsRosBridge
{
public:
  NnsRosBridge (const char *node_name, const char *topic_name);
  gboolean publish (const guint num_tensors,
      const GstTensorMemory *tensors_mem);
  gboolean setPubTopicInfo (const GstTensorsConfig *conf);
private:
  NnsRosBridge ();
  ~NnsRosBridge ();

  // Variables for ROS configuration
  ros::NodeHandle *nh_parent;
  ros::NodeHandle *nh_child;
  ros::Publisher  ros_sink_pub;
  std::string str_node_name;
  std::string str_pub_topic_name;
  // Variables for publish ROS topic
  gboolean ready_to_pub;
  guint num_of_tensors_pub;
  std_msgs::MultiArrayLayout topic_layouts_pub[NNS_TENSOR_SIZE_LIMIT];
};

extern "C"
{
#endif /* __cplusplus */

void *nns_ros_bridge_init (const char *node_name, const char *topic_name);
gboolean nns_ros_bridge_publish (void *instance, const guint num_tensors,
    const GstTensorMemory *tensors_mem);
gboolean nns_ros_bridge_set_pub_topic (void *instance,
    const GstTensorsConfig *conf);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif /* _NNS_ROS_BRIDGE_H_ */
