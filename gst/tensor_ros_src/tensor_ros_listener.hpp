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
    int payload_size;           /*<< The payload size of Ros message */

  public:
    RosListener (GstTensorRosSrc *rossrc) {
      this->rossrc = rossrc;
      this->payload_size = rossrc->count * tensor_element_size[rossrc->datatype];
    }
};

#define INIT_ROSLISTENER(DATATYPE)  \
  class DATATYPE##RosListener : public RosListener { \
    public: \
      DATATYPE##RosListener (GstTensorRosSrc *rossrc) : RosListener (rossrc) { } \
      void Callback(const std_msgs::DATATYPE##MultiArray msg) { \
        gpointer queue_item = g_malloc0 (this->payload_size); \
        std::memcpy (queue_item, msg.data.data(), this->payload_size); \
        g_async_queue_push (this->rossrc->queue, queue_item); \
        GST_DEBUG_OBJECT (this->rossrc, "Queue size: %d\n", g_async_queue_length (this->rossrc->queue)); \
      } \
  }; \

#endif /* __TENSOR_ROS_LISTENER_H__ */
