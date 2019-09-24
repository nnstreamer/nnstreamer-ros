/**
 * Copyright (C) 2019 Samsung Electronics Co., Ltd. All rights reserved.
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
 * @file   tensor_ros_lintener.hpp
 * @author Sangjung Woo <sangjung.woo@samsung.com>
 * @date   03/06/2019
 * @brief  A set of Ros Listener classes for various types
 *
 * @bug     No known bugs except for NYI items
 */

#ifndef __TENSOR_ROS_LISTENER_H__
#define __TENSOR_ROS_LISTENER_H__

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include "tensor_ros_src.h"

/**
 * @brief Basse Ros Listener class
 */
class RosListener {
  protected:
    GstTensorRosSrc *rossrc;    /*<< GstTensorRosSrc instance */
    gboolean is_initialized;

  public:
    RosListener (GstTensorRosSrc *rossrc) {
      this->rossrc = rossrc;
      this->is_initialized = false;
    }
};

#define INIT_ROSLISTENER(DATATYPE)  \
  class DATATYPE##RosListener : public RosListener { \
    public: \
      DATATYPE##RosListener (GstTensorRosSrc *rossrc) : RosListener (rossrc) { } \
      void Callback(const std_msgs::DATATYPE##MultiArray msg) { \
        gsize payload_size = msg.layout.dim[0].stride * gst_tensor_get_element_size (rossrc->datatype); \
        g_assert (payload_size != 0); \
        \
        if (!this->is_initialized) { \
          gint i; \
          this->rossrc->payload_size = payload_size; \
          \
          /* Set TensorConfig based on Ros message */ \
          for (i = 0; i < NNS_TENSOR_RANK_LIMIT; ++i) \
            this->rossrc->config.info.dimension[i] = 1; \
          \
          for (i = 0; i < NNS_TENSOR_RANK_LIMIT; ++i) { \
            if (msg.layout.dim[i].size == 1) \
            break; \
          this->rossrc->config.info.dimension[i] = msg.layout.dim[i].size; \
          } \
          this->rossrc->config.info.type =  this->rossrc->datatype; \
          this->rossrc->configured = true; \
          this->is_initialized = true; \
        } \
        /* After NNstreamer pipeline is setup and playing, \
        the payload size of Ros message could not be changed */ \
        g_assert (payload_size == this->rossrc->payload_size); \
        \
        gpointer queue_item = g_malloc0 (payload_size); \
        std::memcpy (queue_item, msg.data.data(), payload_size); \
        g_async_queue_push (this->rossrc->queue, queue_item); \
        \
        silent_debug (this->rossrc, "payload_size: %" G_GSIZE_FORMAT " bytes\n", payload_size); \
      } \
  }; \

#endif /* __TENSOR_ROS_LISTENER_H__ */
