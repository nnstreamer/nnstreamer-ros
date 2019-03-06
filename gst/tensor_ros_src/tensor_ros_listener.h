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
 * @file   tensor_ros_lintener.h
 * @author Sangjung Woo <sangjung.woo@samsung.com>
 * @date   03/06/2019
 * @brief  A set of Ros Listener classes for various types
 *
 * @bug     No known bugs except for NYI items
 */

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

#include "tensor_ros_src.h"

/**
 * @brief Ros Listener class for Int32 type
 */
class Int32RosListener {
  private:
    GstTensorRosSrc *rossrc;    /*<< GstTensorRosSrc instance */
    int payload_size;           /*<< The payload size of Ros message */

  public:
    Int32RosListener (GstTensorRosSrc *rossrc);
    void Callback(const std_msgs::Int32MultiArray msg);
};
