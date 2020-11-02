/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * Copyright (C) 2019 Sangjung Woo <sangjung.woo@samsung.com>
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *
 * @file        tensor_ros_src.h
 * @date        10/20/2020
 * @brief       GStreamer plugin to subscribe a Ros topic and convert them into
 *              tensor stream
 * @see         https://github.com/nnstreamer/nnstreamer-ros
 * @author      Sangjung Woo <sangjung.woo@samsung.com>
 *              Wook Song <wook16.song@samsung.com>
 * @bug         No known bugs except for NYI items
 *
 * This class bridges between the NNStreamer (C) and ROS2 frameworks (RCLCPP/C++)
 * by implementing the NnsRosPublisher class.
 *
 */

#ifndef __GST_TENSOR_ROS_SRC_H__
#define __GST_TENSOR_ROS_SRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>

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

  GstTensorsConfig configs;   /** Output Tensors configuration */
  gboolean configured;        /** is configured based on Ros message */
  gboolean silent;
  GThread *thread;            /** ros subscribe thread */

  gchar *topic_name;          /** ROS topic name to subscribe */
  gdouble timeout;            /** Timeout in seconds waiting for the next message to receive */
  gdouble rate;               /** frequency rate to check */

  GAsyncQueue *queue;         /** data queue */
  void *nns_ros_bind_instance;
};

struct _GstTensorRosSrcClass
{
  GstPushSrcClass parent_class;
};

GType gst_ros_src_get_type (void);

G_END_DECLS

#endif /* __GST_TENSOR_ROS_SRC_H__ */
