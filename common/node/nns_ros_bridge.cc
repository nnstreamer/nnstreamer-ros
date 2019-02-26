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
 * @file   nns_ros_bridge.cc
 * @author Wook Song <wook16.song@samsung.com>
 * @date   11/19/2018
 * @brief  A bridge for ROS support within NNStreamer
 *
 * This class bridges between the NNStreamer (C) and ROS frameworks (ROSCPP/C++).
 *
 * @bug     No known bugs.
 */
#include <ros/ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include "nns_ros_bridge.h"
#include "nns_ros_bridge/tensors.h"

const char BASE_NODE_NAME[] = "tensor_ros_sink";
const char SELF_NAME[] = "nns_ros_bridge";
const guint DEFAULT_Q_SIZE = 15;

/**
 * @brief	The only public constructor of this class
 * @param[in] node_name : The name of the ROS node under /tensor_ros_sink
 * @param[in] topic_name : The name of topic what this class concerns
 * @param[in] is_dummy_roscore : If TRUE, create the instance without roscore connection
 * @return None
 */
NnsRosBridge::NnsRosBridge (const char *node_name, const char *topic_name,
    gboolean is_dummy_roscore)
{
  /* ROS initialization requires command-line arguments */
  int dummy_argc = 0;
  char **dummy_argv = NULL;

  this->str_node_name = std::string (node_name);
  this->str_pub_topic_name = std::string (topic_name);
  this->ready_to_pub = FALSE;

  if (!is_dummy_roscore) {
    if (getenv("ROS_MASTER_URI") == NULL)
      throw UNDEFINED_ROS_MASTER_URI;

    ros::init (dummy_argc, dummy_argv, this->str_node_name);

    if (!ros::master::check())
      throw FAILED_TO_CONNECT_ROSCORE;

    this->nh_parent = new ros::NodeHandle (BASE_NODE_NAME);
    this->nh_child = new ros::NodeHandle (*(this->nh_parent),
      this->str_node_name);

    /**
    * The name of published topic would be
    * /tensor_ros_sink/PID_${PID}/${ElementNameOfTensorRosSink}
    */
    this->ros_sink_pub = this->nh_child->advertise<nns_ros_bridge::tensors>(
        this->str_pub_topic_name, DEFAULT_Q_SIZE);
  }
}

/**
 * @brief	A method for configuraing topic information to publish
 * @param[in] conf : The configuration information at the NNStreamer side
 * @return TRUE if the configuration is successfully done
 */
gboolean NnsRosBridge::setPubTopicInfo (const GstTensorsConfig *conf)
{
  const GstTensorsInfo *tensors_info = &conf->info;

  g_return_val_if_fail (tensors_info->num_tensors <= NNS_TENSOR_SIZE_LIMIT,
      FALSE);

  this->num_of_tensors_pub = tensors_info->num_tensors;

  for (guint i = 0; i < this->num_of_tensors_pub; ++i) {
    gsize tensor_size = gst_tensor_info_get_size (&tensors_info->info[i]);
    guint each_dim_stride = tensor_size;

    for (gint j = (NNS_TENSOR_RANK_LIMIT - 1); j >= 0; --j) {
      std_msgs::MultiArrayDimension each_dim;
      /**
       * FIXME: Since the type of 'stride' of std_msgs::MultiArrayDimension is
       * unsigned int,conversion from size_t to unsigned int is requried.
       */
      each_dim.label = tensor_element_typename[tensors_info->info[i].type];
      each_dim.size =
          tensors_info->info[i].dimension[j];
      if (j != (NNS_TENSOR_RANK_LIMIT - 1)) {
        each_dim_stride /= tensors_info->info[i].dimension[j + 1];
      }
      each_dim.stride = each_dim_stride;

      this->topic_layouts_pub[i].dim.push_back (each_dim);
    }

    g_return_val_if_fail (
        this->topic_layouts_pub[i].dim.at(0).stride == tensor_size, FALSE
        );
  }

  this->ready_to_pub = TRUE;

  return ready_to_pub;
}

/**
 * @brief	A method for publishing a ROS topic that contains the tensors passed by the NNStreamer framework
 * @param[in] num_tensors : The number of tensors included in tensors_mem (for the verification purpose)
 * @param[in] tensors_mem : The pointer of containers consists of information and raw data of tensors
 * @return TRUE if the configuration is successfully done
 */
gboolean NnsRosBridge::publish (const guint num_tensors,
    const GstTensorMemory *tensors_mem)
{
  nns_ros_bridge::tensors tensors_msg;

  g_return_val_if_fail (ros::ok(), FALSE);
  g_return_val_if_fail (this->ready_to_pub, FALSE);
  g_printerr ("num_tensors = %u, this->num_of_tensors_pub = %u\n", num_tensors, this->num_of_tensors_pub);
  g_return_val_if_fail (num_tensors == this->num_of_tensors_pub, FALSE);

  for (guint i = 0; i < this->num_of_tensors_pub; ++i) {
    std_msgs::UInt8MultiArray each_tensor;
    uint8_t *src_data = (uint8_t *) tensors_mem[i].data;

    each_tensor.layout = this->topic_layouts_pub[i];
    each_tensor.data.assign (src_data, src_data + tensors_mem[i].size);

    tensors_msg.tensors.push_back (each_tensor);
  }

  if (!this->is_dummy_roscore) {
    this->ros_sink_pub.publish (tensors_msg);
    ros::spinOnce();
  }

  return TRUE;
}

/**
 * C functions for NNStreamer plugins that want to publish or subscribe
 * ROS topics. nns_ros_bridge_init() should be invoked before configuring the
 * tensors information to publish or subscribe. Each NNStreamer plugin which
 * uses this class should be holding the instance of NnsRosBridge (i.e., the
 * void type pointer what the nns_ros_bridge_init() function returns).
 */
void *
nns_ros_bridge_init (const char *node_name, const char *topic_name,
    gboolean is_dummy_roscore)
{
  try {
    return new NnsRosBridge (node_name, topic_name, is_dummy_roscore);
  } catch (const err_code e) {
    switch (e) {
      case UNDEFINED_ROS_MASTER_URI:
        g_critical ("%s: ROS_MASTER_URI is not defined in the environment\n",
            SELF_NAME);
        break;

      case FAILED_TO_CONNECT_ROSCORE:
        g_critical
            ("%s: failed to connect to master: please make sure roscore is running\n",
            SELF_NAME);
        break;

      default:
        break;
    }
    return NULL;
  }
}

gboolean
nns_ros_bridge_publish (void *instance, const guint num_tensors,
    const GstTensorMemory *tensors_mem)
{
  NnsRosBridge *nrb_instance = (NnsRosBridge *) instance;

  return nrb_instance->publish (num_tensors, tensors_mem);
}

gboolean
nns_ros_bridge_set_pub_topic (void *instance, const GstTensorsConfig *conf)
{
  NnsRosBridge *nrb_instance = (NnsRosBridge *) instance;

  return nrb_instance->setPubTopicInfo (conf);
}
