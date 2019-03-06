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
 * @file   tensor_ros_lintener.cc
 * @author Sangjung Woo <sangjung.woo@samsung.com>
 * @date   03/06/2019
 * @brief  A bridge for ROS support within NNStreamer
 *
 * This class bridges between the NNStreamer (C) and ROS frameworks (ROSCPP/C++).
 *
 * @bug     No known bugs.
 */
#include <tensor_typedef.h>

#include "tensor_ros_listener.h"

/**
 * @brief	The public constructor of Int32RosListener
 * @param[in] rossrc : GstTensorRosSrc instance
 * @return None
 */
Int32RosListener::Int32RosListener (GstTensorRosSrc *rossrc)
{
  this->rossrc = rossrc;
  this->payload_size = rossrc->count * tensor_element_size[rossrc->datatype];
}

/**
 * @brief	A Callback method when target Ros topic is published
 * @param[in] msg : Published message
 * @return None
 */
void
Int32RosListener::Callback(const std_msgs::Int32MultiArray msg)
{
  /* Get the payload of message and enqueue them into GAsyncQueue */
  gpointer queue_item = g_malloc0 (this->payload_size);

  std::memcpy (queue_item, msg.data.data(), this->payload_size);
  g_async_queue_push (this->rossrc->queue, queue_item);

  GST_DEBUG_OBJECT (this->rossrc, "Queue size: %d\n", g_async_queue_length (this->rossrc->queue));
}
