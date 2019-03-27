/**
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * Copyright (C) 2019 Samsung Electronics Co., Ltd.
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
 */

/**
 * @file	tensor_ros_src.h
 * @date	03/06/2019
 * @brief	GStreamer plugin to handle tensor stream from Ros topic
 * @see		https://github.com/nnsuite/nnstreamer
 * @see		https://github.com/nnsuite/nnstreamer-ros
 * @author	Sangjung Woo <sangjung.woo@samsung.com>
 * @bug		No known bugs except for NYI items
 */

#ifndef __GST_TENSOR_ROS_SRC_H__
#define __GST_TENSOR_ROS_SRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

#include "nns_ros_subscriber.h"

G_BEGIN_DECLS

#define GST_TYPE_TENSOR_ROS_SRC \
  (gst_tensor_ros_src_get_type())
#define GST_TENSOR_ROS_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_TENSOR_ROS_SRC,GstTensorRosSrc))
#define GST_TENSOR_ROS_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_TENSOR_ROS_SRC,GstTensorRosSrcClass))
#define GST_IS_TENSOR_ROS_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_TENSOR_ROS_SRC))
#define GST_IS_TENSOR_ROS_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_TENSOR_ROS_SRC))

typedef struct _GstTensorRosSrc GstTensorRosSrc;
typedef struct _GstTensorRosSrcClass GstTensorRosSrcClass;

struct _GstTensorRosSrc
{
  GstPushSrc parent;

  GstTensorConfig config; /** Output Tensor configuration */
  gboolean configured;    /** is configured based on Ros message */
  gboolean silent;
  GThread *thread;        /** ros subscribe thread */

  gchar *topic_name;      /** ROS topic name to subscribe */
  tensor_type datatype;   /** Primitive datatype of ROS topic */
  gsize payload_size;     /** Actual payload size of ROS message */
  gulong freq_rate;       /** frequency rate to check */

  GAsyncQueue *queue;     /** data queue */
  class NnsRosSubscriber *ros_sub;    /** NnsRosSubscriber instance for subscribing Ros topic in other thread */
};

struct _GstTensorRosSrcClass 
{
  GstPushSrcClass parent_class;
};

GType gst_ros_src_get_type (void);

G_END_DECLS

#endif /* __GST_TENSOR_ROS_SRC_H__ */
