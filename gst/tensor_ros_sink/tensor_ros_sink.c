/**
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
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
 * SECTION:element-tensor_ros_sink
 *
 * Sink element to publish a ROS topic for tensor stream
 *
 * @file    tensor_ros_sink.c
 * @date    15 June 2018
 * @brief   GStreamer plugin to publish a ROS topic for tensor stream
 * @see     https://github.com/nnsuite/nnstreamer-ros
 * @author  Wook Song <wook16.song@samsung.com>
 * @bug     No known bugs except for NYI items
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <nns_ros_bridge.h>
#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>

#include "tensor_ros_sink.h"

/**
 * @brief Macro for debug mode.
 */
#ifndef DBG
#define DBG (!self->silent)
#endif

/**
 * @brief Macro for debug message.
 */
#define silent_debug(SELF, ...) do { \
    if (DBG) { \
      GST_DEBUG_OBJECT (SELF, __VA_ARGS__); \
    } \
  } while (0)

GST_DEBUG_CATEGORY_STATIC (gst_tensor_ros_sink_debug);
#define GST_CAT_DEFAULT gst_tensor_ros_sink_debug

/**
 * @brief tensor_ros_sink signals.
 */
enum
{
  SIGNAL_NEW_DATA,
  SIGNAL_STREAM_START,
  SIGNAL_EOS,
  LAST_SIGNAL
};

/**
 * @brief tensor_ros_sink properties.
 */
enum
{
  PROP_0,
  PROP_SIGNAL_RATE,
  PROP_EMIT_SIGNAL,
  PROP_SILENT
};

/**
 * @brief Flag to emit signals.
 */
#define DEFAULT_EMIT_SIGNAL TRUE

/**
 * @brief New data signals per second.
 */
#define DEFAULT_SIGNAL_RATE 0

/**
 * @brief Flag to print minimized log.
 */
#define DEFAULT_SILENT TRUE

/**
 * @brief Flag for qos event.
 *
 * See GstBaseSink::qos property for more details.
 */
#define DEFAULT_QOS TRUE

/**
 * @brief Template for sink pad.
 */
static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_TENSOR_CAP_DEFAULT "; " GST_TENSORS_CAP_DEFAULT));

/**
 * @brief Variable for signal ids.
 */
static guint _tensor_ros_sink_signals[LAST_SIGNAL] = { 0 };

/** GObject method implementation */
static void gst_tensor_ros_sink_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_tensor_ros_sink_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static void gst_tensor_ros_sink_dispose (GObject * object);
static void gst_tensor_ros_sink_finalize (GObject * object);

/** GstBaseSink method implementation */
static gboolean gst_tensor_ros_sink_start (GstBaseSink * sink);
static gboolean gst_tensor_ros_sink_stop (GstBaseSink * sink);
static gboolean gst_tensor_ros_sink_event (GstBaseSink * sink,
    GstEvent * event);
static gboolean gst_tensor_ros_sink_query (GstBaseSink * sink,
    GstQuery * query);
static GstFlowReturn gst_tensor_ros_sink_render (GstBaseSink * sink,
    GstBuffer * buffer);
static GstFlowReturn gst_tensor_ros_sink_render_list (GstBaseSink * sink,
    GstBufferList * buffer_list);
static gboolean gst_tensor_ros_sink_set_caps (GstBaseSink * sink,
    GstCaps * caps);
static GstCaps *gst_tensor_ros_sink_get_caps (GstBaseSink * sink,
    GstCaps * filter);

/** internal functions */
static gboolean gst_tensor_ros_sink_render_buffer (GstTensorRosSink * self,
    GstBuffer * inbuf);
static void gst_tensor_ros_sink_set_last_render_time (GstTensorRosSink * self,
    GstClockTime now);
static GstClockTime gst_tensor_ros_sink_get_last_render_time (GstTensorRosSink *
    self);
static void gst_tensor_ros_sink_set_signal_rate (GstTensorRosSink * self,
    guint rate);
static guint gst_tensor_ros_sink_get_signal_rate (GstTensorRosSink * self);
static void gst_tensor_ros_sink_set_emit_signal (GstTensorRosSink * self,
    gboolean emit);
static gboolean gst_tensor_ros_sink_get_emit_signal (GstTensorRosSink * self);
static void gst_tensor_ros_sink_set_silent (GstTensorRosSink * self,
    gboolean silent);
static gboolean gst_tensor_ros_sink_get_silent (GstTensorRosSink * self);

#define gst_tensor_ros_sink_parent_class parent_class
G_DEFINE_TYPE (GstTensorRosSink, gst_tensor_ros_sink, GST_TYPE_BASE_SINK);

/**
 * @brief Initialize tensor_ros_sink class.
 */
static void
gst_tensor_ros_sink_class_init (GstTensorRosSinkClass *klass)
{
  GObjectClass *gobject_class;
  GstElementClass *element_class;
  GstBaseSinkClass *bsink_class;

  gobject_class = G_OBJECT_CLASS (klass);
  element_class = GST_ELEMENT_CLASS (klass);
  bsink_class = GST_BASE_SINK_CLASS (klass);

  /** GObject methods */
  gobject_class->set_property = gst_tensor_ros_sink_set_property;
  gobject_class->get_property = gst_tensor_ros_sink_get_property;
  gobject_class->dispose = gst_tensor_ros_sink_dispose;
  gobject_class->finalize = gst_tensor_ros_sink_finalize;

  /**
   * GstTensorRosSink::signal-rate:
   *
   * The number of new data signals per second (Default 0 for unlimited, MAX 500)
   * If signal-rate is larger than 0, GstTensorRosSink calculates the time to emit a signal with this property.
   * If set 0 (default value), all the received buffers will be passed to the application.
   *
   * Please note that this property does not guarantee the periodic signals.
   * This means if GstTensorRosSink cannot get the buffers in time, it will pass all the buffers. (working like default 0)
   */
  g_object_class_install_property (gobject_class, PROP_SIGNAL_RATE,
      g_param_spec_uint ("signal-rate", "Signal rate",
          "New data signals per second (0 for unlimited, max 500)", 0, 500,
          DEFAULT_SIGNAL_RATE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  /**
   * GstTensorRosSink::emit-signal:
   *
   * The flag to emit the signals for new data, stream start, and eos.
   */
  g_object_class_install_property (gobject_class, PROP_EMIT_SIGNAL,
      g_param_spec_boolean ("emit-signal", "Emit signal",
          "Emit signal for new data, stream start, eos", DEFAULT_EMIT_SIGNAL,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  /**
   * GstTensorRosSink::silent:
   *
   * The flag to enable/disable debugging messages.
   */
  g_object_class_install_property (gobject_class, PROP_SILENT,
      g_param_spec_boolean ("silent", "Silent", "Produce verbose output",
          DEFAULT_SILENT, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  /**
   * GstTensorRosSink::new-data:
   *
   * Signal to get the buffer from GstTensorRosSink.
   */
  _tensor_ros_sink_signals[SIGNAL_NEW_DATA] =
      g_signal_new ("new-data", G_TYPE_FROM_CLASS (klass), G_SIGNAL_RUN_LAST,
      G_STRUCT_OFFSET (GstTensorRosSinkClass, new_data), NULL, NULL, NULL,
      G_TYPE_NONE, 1, GST_TYPE_BUFFER | G_SIGNAL_TYPE_STATIC_SCOPE);

  /**
   * GstTensorRosSink::stream-start:
   *
   * Signal to indicate the start of a new stream.
   * Optional. An application can use this signal to detect the start of a new stream, instead of the message GST_MESSAGE_STREAM_START from pipeline.
   */
  _tensor_ros_sink_signals[SIGNAL_STREAM_START] =
      g_signal_new ("stream-start", G_TYPE_FROM_CLASS (klass),
      G_SIGNAL_RUN_LAST, G_STRUCT_OFFSET (GstTensorRosSinkClass, stream_start),
      NULL, NULL, NULL, G_TYPE_NONE, 0, G_TYPE_NONE);

  /**
   * GstTensorRosSink::eos:
   *
   * Signal to indicate the end-of-stream.
   * Optional. An application can use this signal to detect the EOS (end-of-stream), instead of the message GST_MESSAGE_EOS from pipeline.
   */
  _tensor_ros_sink_signals[SIGNAL_EOS] =
      g_signal_new ("eos", G_TYPE_FROM_CLASS (klass), G_SIGNAL_RUN_LAST,
      G_STRUCT_OFFSET (GstTensorRosSinkClass, eos), NULL, NULL, NULL,
      G_TYPE_NONE, 0, G_TYPE_NONE);

  gst_element_class_set_static_metadata (element_class,
      "TensorRosSink",
      "Sink/Tensor",
      "Sink element to publish a ROS topic for tensor stream",
      "Samsung Electronics Co., Ltd.");

  /** pad template */
  gst_element_class_add_static_pad_template (element_class, &sink_template);

  /** GstBaseSink methods */
  bsink_class->start = GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_start);
  bsink_class->stop = GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_stop);
  bsink_class->event = GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_event);
  bsink_class->query = GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_query);
  bsink_class->render = GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_render);
  bsink_class->render_list =
      GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_render_list);
  bsink_class->set_caps = GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_set_caps);
  bsink_class->get_caps = GST_DEBUG_FUNCPTR (gst_tensor_ros_sink_get_caps);
}

/**
 * @brief Initialize tensor_ros_sink element.
 */
static void
gst_tensor_ros_sink_init (GstTensorRosSink *self)
{
  GstBaseSink *bsink;

  bsink = GST_BASE_SINK (self);

  g_mutex_init (&self->mutex);

  /** init properties */
  self->silent = DEFAULT_SILENT;
  self->emit_signal = DEFAULT_EMIT_SIGNAL;
  self->signal_rate = DEFAULT_SIGNAL_RATE;
  self->last_render_time = GST_CLOCK_TIME_NONE;
  self->in_caps = NULL;

  /** enable qos */
  gst_base_sink_set_qos_enabled (bsink, DEFAULT_QOS);
}

/**
 * @brief Setter for tensor_ros_sink properties.
 *
 * GObject method implementation.
 */
static void
gst_tensor_ros_sink_set_property (GObject *object, guint prop_id,
    const GValue *value, GParamSpec *pspec)
{
  GstTensorRosSink *self;

  self = GST_TENSOR_ROS_SINK (object);

  switch (prop_id) {
    case PROP_SIGNAL_RATE:
      gst_tensor_ros_sink_set_signal_rate (self, g_value_get_uint (value));
      break;

    case PROP_EMIT_SIGNAL:
      gst_tensor_ros_sink_set_emit_signal (self, g_value_get_boolean (value));
      break;

    case PROP_SILENT:
      gst_tensor_ros_sink_set_silent (self, g_value_get_boolean (value));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/**
 * @brief Getter for tensor_ros_sink properties.
 *
 * GObject method implementation.
 */
static void
gst_tensor_ros_sink_get_property (GObject *object, guint prop_id,
    GValue *value, GParamSpec *pspec)
{
  GstTensorRosSink *self;

  self = GST_TENSOR_ROS_SINK (object);

  switch (prop_id) {
    case PROP_SIGNAL_RATE:
      g_value_set_uint (value, gst_tensor_ros_sink_get_signal_rate (self));
      break;

    case PROP_EMIT_SIGNAL:
      g_value_set_boolean (value, gst_tensor_ros_sink_get_emit_signal (self));
      break;

    case PROP_SILENT:
      g_value_set_boolean (value, gst_tensor_ros_sink_get_silent (self));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/**
 * @brief Function to drop all references.
 *
 * GObject method implementation.
 */
static void
gst_tensor_ros_sink_dispose (GObject *object)
{
  GstTensorRosSink *self;

  self = GST_TENSOR_ROS_SINK (object);

  g_mutex_lock (&self->mutex);
  gst_caps_replace (&self->in_caps, NULL);
  g_mutex_unlock (&self->mutex);

  G_OBJECT_CLASS (parent_class)->dispose (object);
}

/**
 * @brief Function to finalize instance.
 *
 * GObject method implementation.
 */
static void
gst_tensor_ros_sink_finalize (GObject *object)
{
  GstTensorRosSink *self;

  self = GST_TENSOR_ROS_SINK (object);

  g_mutex_clear (&self->mutex);

  G_OBJECT_CLASS (parent_class)->finalize (object);
}

/**
 * @brief Start processing, called when state changed null to ready.
 *
 * GstBaseSink method implementation.
 */
static gboolean
gst_tensor_ros_sink_start (GstBaseSink *sink)
{
  gint32 pid = getpid ();
  gchar *str_pid = g_strdup_printf (format_node_name_pid, pid);
  GstTensorRosSink *self = GST_TENSOR_ROS_SINK (sink);
  self->nns_ros_bind_instance =
      nns_ros_bridge_init (str_pid, GST_ELEMENT_NAME (GST_ELEMENT (sink)));
  g_free (str_pid);

  if (self->nns_ros_bind_instance == NULL)
    return FALSE;
  return TRUE;
}

/**
 * @brief Stop processing, called when state changed ready to null.
 *
 * GstBaseSink method implementation.
 */
static gboolean
gst_tensor_ros_sink_stop (GstBaseSink *sink)
{
  /** free resources */
  return TRUE;
}

/**
 * @brief Handle events.
 *
 * GstBaseSink method implementation.
 */
static gboolean
gst_tensor_ros_sink_event (GstBaseSink *sink, GstEvent *event)
{
  GstTensorRosSink *self;
  GstEventType type;

  self = GST_TENSOR_ROS_SINK (sink);
  type = GST_EVENT_TYPE (event);

  silent_debug (self, "received event %s", GST_EVENT_TYPE_NAME (event));

  switch (type) {
    case GST_EVENT_STREAM_START:
      if (gst_tensor_ros_sink_get_emit_signal (self)) {
        g_signal_emit (self, _tensor_ros_sink_signals[SIGNAL_STREAM_START], 0);
      }
      break;

    case GST_EVENT_EOS:
      if (gst_tensor_ros_sink_get_emit_signal (self)) {
        g_signal_emit (self, _tensor_ros_sink_signals[SIGNAL_EOS], 0);
      }
      break;

    default:
      break;
  }

  return GST_BASE_SINK_CLASS (parent_class)->event (sink, event);
}

/**
 * @brief Handle queries.
 *
 * GstBaseSink method implementation.
 */
static gboolean
gst_tensor_ros_sink_query (GstBaseSink *sink, GstQuery *query)
{
  GstTensorRosSink *self;
  GstQueryType type;
  GstFormat format;

  self = GST_TENSOR_ROS_SINK (sink);
  type = GST_QUERY_TYPE (query);

  silent_debug (self, "received query %s", GST_QUERY_TYPE_NAME (query));

  switch (type) {
    case GST_QUERY_SEEKING:
      /** tensor sink does not support seeking */
      gst_query_parse_seeking (query, &format, NULL, NULL, NULL);
      gst_query_set_seeking (query, format, FALSE, 0, -1);
      return TRUE;

    default:
      break;
  }

  return GST_BASE_SINK_CLASS (parent_class)->query (sink, query);
}

/**
 * @brief Handle buffer.
 *
 * GstBaseSink method implementation.
 */
static GstFlowReturn
gst_tensor_ros_sink_render (GstBaseSink *sink, GstBuffer *buffer)
{
  GstTensorRosSink *self;

  self = GST_TENSOR_ROS_SINK (sink);
  g_return_val_if_fail (gst_tensor_ros_sink_render_buffer (self, buffer),
      GST_FLOW_ERROR);

  return GST_FLOW_OK;
}

/**
 * @brief Handle list of buffers.
 *
 * GstBaseSink method implementation.
 */
static GstFlowReturn
gst_tensor_ros_sink_render_list (GstBaseSink *sink, GstBufferList *buffer_list)
{
  GstTensorRosSink *self;
  GstBuffer *buffer;

  guint i;
  guint num_buffers;

  self = GST_TENSOR_ROS_SINK (sink);
  num_buffers = gst_buffer_list_length (buffer_list);
  for (i = 0; i < num_buffers; i++) {
    buffer = gst_buffer_list_get (buffer_list, i);
    g_return_val_if_fail (gst_tensor_ros_sink_render_buffer (self, buffer),
        GST_FLOW_ERROR);
  }

  return GST_FLOW_OK;
}

/**
 * @brief Read cap, parse tensor configuration (dim/type) from the cap.
 * @param[in] caps The input caps to be read
 * @param[out] config configured tensor info
 * @return TRUE if successful (both dim/type read). FALSE if not.
 */
static gboolean
gst_tensor_ros_sink_read_caps (const GstCaps *caps, GstTensorsConfig *config)
{
  GstStructure *structure;

  g_return_val_if_fail (config != NULL, FALSE);

  structure = gst_caps_get_structure (caps, 0);

  if (!gst_structure_has_name (structure, "other/tensor") &&
      !gst_structure_has_name (structure, "other/tensors")) {
    GST_ERROR ("The input caps, %s, "
        "do not match the capabilities of the sink pad\n",
        gst_structure_get_name (structure));
    return FALSE;
  }

  gst_tensors_config_from_structure (config, structure);

  return gst_tensors_info_validate (&config->info);
}

/**
 * @brief Funtion for new caps.
 *
 * GstBaseSink method implementation.
 */
static gboolean
gst_tensor_ros_sink_set_caps (GstBaseSink *sink, GstCaps *caps)
{
  GstTensorRosSink *self;
  GstTensorsConfig in_conf;

  self = GST_TENSOR_ROS_SINK (sink);

  g_mutex_lock (&self->mutex);
  gst_caps_replace (&self->in_caps, caps);
  g_mutex_unlock (&self->mutex);

  if (!gst_tensor_ros_sink_read_caps (self->in_caps, &in_conf)) {
    GST_ERROR_OBJECT (self, "Failed to read the input caps\n");
    return FALSE;
  }

  if (!gst_tensors_config_validate (&in_conf)) {
    GST_ERROR_OBJECT (self, "Failed to validate the input caps\n");
    return FALSE;
  }

  self->in_config = in_conf;
  nns_ros_bridge_set_pub_topic (self->nns_ros_bind_instance, &self->in_config);

  return TRUE;
}

/**
 * @brief Funtion to return caps of subclass.
 *
 * GstBaseSink method implementation.
 */
static GstCaps *
gst_tensor_ros_sink_get_caps (GstBaseSink *sink, GstCaps *filter)
{
  GstTensorRosSink *self;
  GstCaps *caps;

  self = GST_TENSOR_ROS_SINK (sink);

  g_mutex_lock (&self->mutex);
  caps = self->in_caps;

  if (caps) {
    if (filter) {
      caps = gst_caps_intersect_full (filter, caps, GST_CAPS_INTERSECT_FIRST);
    } else {
      gst_caps_ref (caps);
    }
  }
  g_mutex_unlock (&self->mutex);

  return caps;
}

/**
 * @brief Handle buffer data.
 * @return None
 * @param self pointer to GstTensorRosSink
 * @param buffer pointer to GstBuffer to be handled
 */
static gboolean
gst_tensor_ros_sink_render_buffer (GstTensorRosSink *self, GstBuffer *inbuf)
{
  GstTensorMemory in_tensors[NNS_TENSOR_SIZE_LIMIT];
  GstMapInfo in_info[NNS_TENSOR_SIZE_LIMIT];
  GstMemory *in_mem[NNS_TENSOR_SIZE_LIMIT];
  guint signal_rate;
  guint num_in_tensors;
  guint i;

  gboolean notify = FALSE;
  GstClockTime now = GST_CLOCK_TIME_NONE;

  g_return_val_if_fail (GST_IS_TENSOR_ROS_SINK (self), FALSE);

  num_in_tensors = gst_buffer_n_memory (inbuf);

  for (i = 0; i < num_in_tensors; ++i) {
    in_mem[i] = gst_buffer_peek_memory (inbuf, i);
    gst_memory_map (in_mem[i], &in_info[i], GST_MAP_READ);

    in_tensors[i].data = in_info[i].data;
    in_tensors[i].size = in_info[i].size;
    in_tensors[i].type = self->in_config.info.info[i].type;
  }

  if (!nns_ros_bridge_publish (self->nns_ros_bind_instance, num_in_tensors,
      in_tensors)) {
    for (i = 0; i < num_in_tensors; ++i) {
      gst_memory_unmap (in_mem[i], &in_info[i]);
    }
    return FALSE;
  }

  signal_rate = gst_tensor_ros_sink_get_signal_rate (self);

  if (signal_rate) {
    GstClock *clock;
    GstClockTime render_time;
    GstClockTime last_render_time;

    clock = gst_element_get_clock (GST_ELEMENT (self));

    if (clock) {
      now = gst_clock_get_time (clock);
      last_render_time = gst_tensor_ros_sink_get_last_render_time (self);

      /** time for next signal */
      render_time = (1000 / signal_rate) * GST_MSECOND + last_render_time;

      if (!GST_CLOCK_TIME_IS_VALID (last_render_time) ||
          GST_CLOCK_DIFF (now, render_time) <= 0) {
        /** send data after render time, or firstly received buffer */
        notify = TRUE;
      }

      gst_object_unref (clock);
    }
  } else {
    /** send data if signal rate is 0 */
    notify = TRUE;
  }

  if (notify) {
    gst_tensor_ros_sink_set_last_render_time (self, now);

    if (gst_tensor_ros_sink_get_emit_signal (self)) {
      silent_debug (self,
          "signal for new data [%" GST_TIME_FORMAT "], rate [%d]",
          GST_TIME_ARGS (now), signal_rate);
      g_signal_emit (self, _tensor_ros_sink_signals[SIGNAL_NEW_DATA], 0, inbuf);
    }
  }

  for (i = 0; i < num_in_tensors; ++i) {
    gst_memory_unmap (in_mem[i], &in_info[i]);
  }

  return TRUE;
}

/**
 * @brief Setter for value last_render_time.
 */
static void
gst_tensor_ros_sink_set_last_render_time (GstTensorRosSink *self,
    GstClockTime now)
{
  g_return_if_fail (GST_IS_TENSOR_ROS_SINK (self));

  g_mutex_lock (&self->mutex);
  self->last_render_time = now;
  g_mutex_unlock (&self->mutex);
}

/**
 * @brief Getter for value last_render_time.
 */
static GstClockTime
gst_tensor_ros_sink_get_last_render_time (GstTensorRosSink *self)
{
  GstClockTime last_render_time;

  g_return_val_if_fail (GST_IS_TENSOR_ROS_SINK (self), GST_CLOCK_TIME_NONE);

  g_mutex_lock (&self->mutex);
  last_render_time = self->last_render_time;
  g_mutex_unlock (&self->mutex);

  return last_render_time;
}

/**
 * @brief Setter for value signal_rate.
 */
static void
gst_tensor_ros_sink_set_signal_rate (GstTensorRosSink *self, guint rate)
{
  g_return_if_fail (GST_IS_TENSOR_ROS_SINK (self));

  silent_debug (self, "set signal_rate to %d", rate);
  g_mutex_lock (&self->mutex);
  self->signal_rate = rate;
  g_mutex_unlock (&self->mutex);
}

/**
 * @brief Getter for value signal_rate.
 */
static guint
gst_tensor_ros_sink_get_signal_rate (GstTensorRosSink *self)
{
  guint rate;

  g_return_val_if_fail (GST_IS_TENSOR_ROS_SINK (self), 0);

  g_mutex_lock (&self->mutex);
  rate = self->signal_rate;
  g_mutex_unlock (&self->mutex);

  return rate;
}

/**
 * @brief Setter for flag emit_signal.
 */
static void
gst_tensor_ros_sink_set_emit_signal (GstTensorRosSink *self, gboolean emit)
{
  g_return_if_fail (GST_IS_TENSOR_ROS_SINK (self));

  silent_debug (self, "set emit_signal to %d", emit);
  g_mutex_lock (&self->mutex);
  self->emit_signal = emit;
  g_mutex_unlock (&self->mutex);
}

/**
 * @brief Getter for flag emit_signal.
 */
static gboolean
gst_tensor_ros_sink_get_emit_signal (GstTensorRosSink *self)
{
  gboolean res;

  g_return_val_if_fail (GST_IS_TENSOR_ROS_SINK (self), FALSE);

  g_mutex_lock (&self->mutex);
  res = self->emit_signal;
  g_mutex_unlock (&self->mutex);

  return res;
}

/**
 * @brief Setter for flag silent.
 */
static void
gst_tensor_ros_sink_set_silent (GstTensorRosSink *self, gboolean silent)
{
  g_return_if_fail (GST_IS_TENSOR_ROS_SINK (self));

  silent_debug (self, "set silent to %d", silent);
  self->silent = silent;
}

/**
 * @brief Getter for flag silent.
 */
static gboolean
gst_tensor_ros_sink_get_silent (GstTensorRosSink *self)
{
  g_return_val_if_fail (GST_IS_TENSOR_ROS_SINK (self), TRUE);

  return self->silent;
}

/**
 * @brief Function to initialize the plugin.
 *
 * See GstPluginInitFunc() for more details.
 */
gboolean gst_tensor_ros_sink_plugin_init (GstPlugin *plugin)
{
  GST_DEBUG_CATEGORY_INIT (gst_tensor_ros_sink_debug, "tensor_ros_sink",
      0, "tensor_ros_sink element");

  return gst_element_register (plugin, "tensor_ros_sink",
      GST_RANK_NONE, GST_TYPE_TENSOR_ROS_SINK);
}

#ifndef PACKAGE
#define PACKAGE "nnstreamer-ros"
#endif

/**
 * @brief Macro to define the entry point of the plugin.
 */
GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    tensor_ros_sink,
    "Sink element to handle tensor stream",
    gst_tensor_ros_sink_plugin_init, VERSION, "LGPL", "nnstreamer-ros",
    "https://github.com/nnsuite/nnstreamer-ros");
