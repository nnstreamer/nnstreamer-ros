/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *
 * @file    tensor_ros_sink.h
 * @date    28 Oct 2020
 * @brief   GStreamer plugin to publish a ROS topic for tensor stream
 * @see     https://github.com/nnstreamer/nnstreamer-ros
 * @author  Wook Song <wook16.song@samsung.com>
 * @bug     No known bugs except for NYI items
 */

#ifndef __GST_TENSOR_ROS_SINK_H__
#define __GST_TENSOR_ROS_SINK_H__

#include <gst/base/gstbasesink.h>
#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>

G_BEGIN_DECLS

#define GST_TYPE_TENSOR_ROS_SINK \
  (gst_tensor_ros_sink_get_type())
#define GST_TENSOR_ROS_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_TENSOR_ROS_SINK,GstTensorRosSink))
#define GST_TENSOR_ROS_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_TENSOR_ROS_SINK,GstTensorRosSinkClass))
#define GST_IS_TENSOR_ROS_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_TENSOR_ROS_SINK))
#define GST_IS_TENSOR_ROS_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_TENSOR_ROS_SINK))

typedef struct _GstTensorRosSink GstTensorRosSink;
typedef struct _GstTensorRosSinkClass GstTensorRosSinkClass;

/* Experimental: To use getpid() */
#ifdef G_OS_WIN32
#include <process.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

const gchar format_node_name_pid[] = "PID_%d";

/**
 * @brief GstTensorRosSink data structure.
 *
 * GstTensorRosSink inherits GstBaseSink.
 */
struct _GstTensorRosSink
{
  GstBaseSink element; /**< parent object */

  GMutex mutex; /**< mutex for processing */
  gboolean silent; /**< true to print minimized log */
  gboolean dummy; /**< true to work without roscore */
  gboolean emit_signal; /**< true to emit signal for new data, eos */
  gboolean save_rosbag; /**< true to save contents of the topic to a rosbag file */
  gchar *location; /**< location of the rosbag file to save */
  void *rosbag_to_save;
  guint signal_rate; /**< new data signals per second */
  GstClockTime last_render_time; /**< buffer rendered time */
  GstCaps *in_caps; /**< received caps */
  GstTensorsConfig in_config;
  void *nns_ros_bind_instance;
};

/**
 * @brief GstTensorRosSinkClass data structure.
 *
 * GstTensorRosSink inherits GstBaseSink.
 */
struct _GstTensorRosSinkClass
{
  GstBaseSinkClass parent_class; /**< parent class */

  /** signals */
  void (*new_data) (GstElement *element, GstBuffer *buffer); /**< signal when new data received */
  void (*stream_start) (GstElement *element); /**< signal when stream started */
  void (*eos) (GstElement *element); /**< signal when end of stream reached */
};

/**
 * @brief Function to get type of tensor_ros_sink.
 */
GType gst_tensor_ros_sink_get_type (void);

G_END_DECLS

#endif /** __GST_TENSOR_ROS_SINK_H__ */
