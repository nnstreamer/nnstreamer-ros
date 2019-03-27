/*
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
 * SECTION:element-tensor_ros_src
 *
 * Source element to publish tensor stream from deginated Ros topic
 *
 * @file    tensor_ros_src.cc
 * @date    03/06/2019
 * @brief   GStreamer plugin to subscribe a Ros topic and convert them into tensor stream
 * @see     https://github.com/nnsuite/nnstreamer-ros
 * @author  Sangjung Woo <sangjung.woo@samsung.com>
 * @bug     No known bugs except for NYI items
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>
#include <string.h>

#include "tensor_ros_src.h"
#include "tensor_ros_listener.hpp"

/**
 * @brief Macro for debug mode.
 */
#ifndef DBG
#define DBG (!rossrc->silent)
#endif

/**
 * @brief Macro for debug message.
 */
#define silent_debug(SELF, ...) do { \
    if (DBG) { \
      GST_DEBUG_OBJECT (SELF, __VA_ARGS__); \
    } \
  } while (0)

GST_DEBUG_CATEGORY_STATIC (gst_tensor_ros_src_debug);
#define GST_CAT_DEFAULT gst_tensor_ros_src_debug

#define OCTET_STREAM_CAPS_STRING   "application/octet-stream"
#define DEFAULT_ROS_QUEUE_SIZE  1000
#define DEFAULT_LIVE_MODE       TRUE

/**
 * @brief Flag to print minimized log.
 */
#define DEFAULT_SILENT TRUE

INIT_ROSLISTENER (Int8);
INIT_ROSLISTENER (UInt8);
INIT_ROSLISTENER (Int16);
INIT_ROSLISTENER (UInt16);
INIT_ROSLISTENER (Int32);
INIT_ROSLISTENER (UInt32);
INIT_ROSLISTENER (Int64);
INIT_ROSLISTENER (UInt64);
INIT_ROSLISTENER (Float32);
INIT_ROSLISTENER (Float64);

/** ROS Listener instance for each type */
static Int32RosListener *int32RosListener;
static UInt32RosListener *uint32RosListener;
static Int8RosListener *int8RosListener;
static UInt8RosListener *uint8RosListener;
static Int16RosListener *int16RosListener;
static UInt16RosListener *uint16RosListener;
static Int64RosListener *int64RosListener;
static UInt64RosListener *uint64RosListener;
static Float32RosListener *float32RosListener;
static Float64RosListener *float64RosListener;

/**
 * @brief Concrete Ros Subscriber class
 */
class TensorRosSub : public NnsRosSubscriber {
  private:
    GstTensorRosSrc *rossrc;

  public:
    TensorRosSub (const gchar *node_name,
      const gchar *topic_name,
      GstTensorRosSrc *rossrc,
      const gulong rate_usec = G_USEC_PER_SEC,
      int argc = 0,
      gchar **argv = NULL) : NnsRosSubscriber (node_name, topic_name, rate_usec, argc, argv)
      {
        this->rossrc = rossrc;
      }

      virtual int RegisterCallback (ros::NodeHandle *nh, ros::Subscriber *sub);
};

/**
 * @brief register callback function to subscribe ROS topic
 */
int
TensorRosSub::RegisterCallback (ros::NodeHandle *nh, ros::Subscriber *sub)
{
  silent_debug (this->rossrc, "[%s]", __func__);

  switch (this->rossrc->datatype) {
    case _NNS_UINT32:
      uint32RosListener = new UInt32RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &UInt32RosListener::Callback, uint32RosListener);
      break;

    case _NNS_INT8:
      int8RosListener = new Int8RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &Int8RosListener::Callback, int8RosListener);
      break;

    case _NNS_UINT8:
      uint8RosListener = new UInt8RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &UInt8RosListener::Callback, uint8RosListener);
      break;

    case _NNS_INT16:
      int16RosListener = new Int16RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &Int16RosListener::Callback, int16RosListener);
      break;

    case _NNS_UINT16:
      uint16RosListener = new UInt16RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &UInt16RosListener::Callback, uint16RosListener);
      break;

    case _NNS_INT64:
      int64RosListener = new Int64RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &Int64RosListener::Callback, int64RosListener);
      break;

    case _NNS_UINT64:
      uint64RosListener = new UInt64RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &UInt64RosListener::Callback, uint64RosListener);
      break;

    case _NNS_FLOAT32:
      float32RosListener = new Float32RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &Float32RosListener::Callback, float32RosListener);
      break;

    case _NNS_FLOAT64:
      float64RosListener = new Float64RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE,
        &Float64RosListener::Callback, float64RosListener);
      break;

    case _NNS_INT32:
    default:
      int32RosListener = new Int32RosListener (this->rossrc);
      *sub = nh->subscribe (this->topic_name, DEFAULT_ROS_QUEUE_SIZE, 
        &Int32RosListener::Callback, int32RosListener);
      break;

  }
  return 0;
}

/**
 * @brief tensor_ros_src properties
 */
enum
{
  PROP_0,
  PROP_SILENT,      /*<< Slient mode for debug */
  PROP_TOPIC,       /*<< ROS topic name to subscribe */
  PROP_FREQ_RATE,   /*<< frequency rate to check the published topic */
  PROP_DATATYPE,    /*<< Primitive datatype of ROS topic */
};

/**
 * @brief Template for ros source pad
 */
static GstStaticPadTemplate src_pad_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_TENSOR_CAP_DEFAULT ";" OCTET_STREAM_CAPS_STRING));

#define gst_tensor_ros_src_parent_class parent_class
G_DEFINE_TYPE (GstTensorRosSrc, gst_tensor_ros_src, GST_TYPE_PUSH_SRC);

/** GObject method implementation */
static void gst_tensor_ros_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_tensor_ros_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static void gst_tensor_ros_src_dispose (GObject * object);

/** GstBaseSrc method implementation */
static gboolean
gst_tensor_ros_src_start (GstBaseSrc * src);
static gboolean
gst_tensor_ros_src_stop (GstBaseSrc * src);
static GstCaps *
gst_tensor_ros_src_get_caps (GstBaseSrc * src, GstCaps * filter);
static gboolean
gst_tensor_ros_src_set_caps (GstBaseSrc * src, GstCaps * caps);
static GstCaps *
gst_tensor_ros_src_fixate (GstBaseSrc * src, GstCaps * caps);

/** GstPushSrc method implementation */
static GstFlowReturn 
gst_tensor_ros_src_create (GstPushSrc * src, GstBuffer ** buffer);

/**
 * @brief The primitive type of Ros
 */
static const gchar* str_type[] = {"int32", "uint32",
  "int16", "uint16",
  "int8", "uint8",
  "float64", "float32",
  "int64", "uint64"};

/**
 * @brief Get tensor_type enum value from the type name
 */
static tensor_type
get_tensor_type (const gchar * type_name)
{
  int type_count = sizeof(str_type);
  for (int i = 0; i < type_count; ++i) {
    if (!g_strcmp0 (type_name, str_type[i]))
      return static_cast<tensor_type> (i);
  }
  return _NNS_END;
}

/**
 * @brief Get the type name from tensor_type enum value
 */
static const gchar *
get_string_type (tensor_type type)
{
  return str_type[type];
}

/**
 *  @brief Set the live opreation mode for live source
 */
static void
set_live_mode (GstBaseSrc * src, gboolean mode)
{
  if (gst_base_src_is_live (src) == mode)
    return ;

  gst_base_src_set_live (src, mode);
  gst_base_src_set_do_timestamp (src, mode);
}

/**
 * @brief initialize the rossrc's class
 */
static void
gst_tensor_ros_src_class_init (GstTensorRosSrcClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *gstbasesrc_class = GST_BASE_SRC_CLASS (klass);
  GstPushSrcClass *gstpushsrc_class = GST_PUSH_SRC_CLASS (klass);

  /* GObject method */
  gobject_class->set_property = gst_tensor_ros_src_set_property;
  gobject_class->get_property = gst_tensor_ros_src_get_property;
  gobject_class->dispose = gst_tensor_ros_src_dispose;  

  /* GstBaseSrc method */
  gstbasesrc_class->start = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_start);
  gstbasesrc_class->stop = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_stop);
  gstbasesrc_class->set_caps = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_set_caps);
  gstbasesrc_class->get_caps = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_get_caps);
  gstbasesrc_class->fixate = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_fixate);

  /* GstPushSrc method */
  gstpushsrc_class->create = gst_tensor_ros_src_create;

  /* Add property */
  g_object_class_install_property (gobject_class, PROP_SILENT,
    g_param_spec_boolean ("silent", "Silent", "Produce verbose output ?",
        FALSE, G_PARAM_READWRITE));
  
  g_object_class_install_property (gobject_class, PROP_TOPIC,
    g_param_spec_string ("topic", "Topic",
      "ROS Topic Name for subscription", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_FREQ_RATE,
    g_param_spec_uint64 ("freqrate", "Frequency_Rate",
      "Frequency rate for checking Ros Topic(Hz, Default: 1Hz)",
      1, G_USEC_PER_SEC, 1, G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_DATATYPE,
    g_param_spec_string ("datatype", "Datatype",
      "Primitive datatype of target ROS topic", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  gst_element_class_add_static_pad_template (gstelement_class,
    &src_pad_template);

  gst_element_class_set_static_metadata (gstelement_class,
    "TensorRosSrc",
    "Source/Tensor",
    "Source element to get the topic data from ROS node",
    "Samsung Electronics Co., Ltd.");
}

/**
 * @brief Initialize ensor_ros_src element
 */
static void
gst_tensor_ros_src_init (GstTensorRosSrc * rossrc)
{
  /* set the default properties */
  rossrc->silent = DEFAULT_SILENT;
  rossrc->topic_name = NULL;
  rossrc->freq_rate = G_USEC_PER_SEC;
  rossrc->queue = g_async_queue_new ();
  rossrc->payload_size = 0;
  rossrc->configured = FALSE;

  /* set live mode as default */
  set_live_mode (GST_BASE_SRC (rossrc), DEFAULT_LIVE_MODE);
  gst_base_src_set_format (GST_BASE_SRC (rossrc), GST_FORMAT_TIME);
}

/**
 * @brief Dispose allocated resources
 */
static void
gst_tensor_ros_src_dispose (GObject * object)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (object);

  if (rossrc->queue)
    g_async_queue_unref (rossrc->queue);

  if (rossrc->topic_name)
    g_free (rossrc->topic_name);

  if (uint32RosListener)
    delete uint32RosListener;

  if (int32RosListener)
    delete int32RosListener;

  if (int8RosListener)
    delete int8RosListener;

  if (uint8RosListener)
    delete uint8RosListener;

  if (int16RosListener)
    delete int16RosListener;

  if (uint16RosListener)
    delete uint16RosListener;

  if (int64RosListener)
    delete int64RosListener;

  if (uint64RosListener)
    delete uint64RosListener;

  if (float32RosListener)
    delete float32RosListener;

  if (float64RosListener)
    delete float64RosListener;

  G_OBJECT_CLASS (parent_class)->dispose (object);
}

/**
 * @brief Start handler
 */
static gboolean
gst_tensor_ros_src_start (GstBaseSrc * src)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);

  silent_debug (rossrc, "start RosSubscriber");
  rossrc->ros_sub = new TensorRosSub ("TensorRosSub", rossrc->topic_name,
    rossrc, rossrc->freq_rate);
  rossrc->ros_sub->Start (&rossrc->thread);

  while (!rossrc->configured)
    g_usleep (G_USEC_PER_SEC / rossrc->freq_rate);

  gst_base_src_start_complete (src, GST_FLOW_OK);
  silent_debug (rossrc, "tensor configuration is completed");
  return TRUE;
}

/**
 * @brief Stop handler
 */
static gboolean
gst_tensor_ros_src_stop (GstBaseSrc * src)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);

  silent_debug (rossrc, "stop RosSubscriber");
  rossrc->ros_sub->RequestStop ();
  return TRUE;
}

/**
 * @brief Get caps handler
 */
static GstCaps *
gst_tensor_ros_src_get_caps (GstBaseSrc * src, GstCaps * filter)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);
  GstCaps *caps;

  caps = gst_pad_get_current_caps (src->srcpad);
  if (caps == NULL)
    caps = gst_pad_get_pad_template_caps (src->srcpad);

  if (filter) {
    GstCaps *intersection;
    intersection = gst_caps_intersect_full (filter, caps, GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (caps);
    caps = intersection;
  }

  silent_debug (rossrc, "caps: %" GST_PTR_FORMAT, caps);
  // gst_caps_replace (&rossrc->caps, caps);

  return caps;
}

/**
 * @brief Set caps handler
 */
static gboolean
gst_tensor_ros_src_set_caps (GstBaseSrc * src, GstCaps * caps)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);

  gst_pad_set_caps (src->srcpad, caps);
  silent_debug (rossrc, "set caps: %" GST_PTR_FORMAT, caps);

  return TRUE;
}

static GstCaps *
gst_tensor_ros_src_fixate (GstBaseSrc * src, GstCaps * caps)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);
  GstCaps *update_caps;
  GstCaps *config_caps = gst_tensor_caps_from_config (&rossrc->config);

  update_caps = gst_caps_intersect (caps, config_caps);
  if (!update_caps) {
    GST_ERROR_OBJECT (rossrc,
      "Only Tensor and application/octet-stream MIME are supported for now");
  }

  gst_caps_unref (caps);
  return gst_caps_fixate (update_caps);
}

/**
 * @brief Setter for tensor_ros_src properties
 */
static void
gst_tensor_ros_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (object);

  switch (prop_id) {
    case PROP_SILENT:
      rossrc->silent = g_value_get_boolean (value);
      silent_debug (rossrc, "Silent Mode: %s\n", rossrc->silent ? "true" : "false");
      break;

    case PROP_TOPIC:
      rossrc->topic_name = g_strdup(g_value_get_string (value));
      silent_debug (rossrc, "topic name: %s\n", rossrc->topic_name);
      break;

    case PROP_FREQ_RATE:
      rossrc->freq_rate = static_cast<gulong>(G_USEC_PER_SEC / g_value_get_uint64 (value));
      silent_debug (rossrc, "Freq Hz: %lu\n", G_USEC_PER_SEC / rossrc->freq_rate);
      break;

    case PROP_DATATYPE:
      rossrc->datatype = get_tensor_type (g_value_get_string (value));
      silent_debug (rossrc, "Datatype: %s\n", get_string_type(rossrc->datatype));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/**
 * @brief Getter for tensor_ros_src properties
 */
static void
gst_tensor_ros_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean (value, rossrc->silent);
      break;
    
    case PROP_TOPIC:
      g_value_set_string (value, rossrc->topic_name);
      break;

    case PROP_FREQ_RATE:
      g_value_set_uint64 (value, G_USEC_PER_SEC / rossrc->freq_rate);
      break;

    case PROP_DATATYPE:
      g_value_set_string (value, get_string_type(rossrc->datatype));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/**
 * @brief Push GstBuffer which contains the subscribed ROS data
 */
static GstFlowReturn
gst_tensor_ros_src_create (GstPushSrc * src, GstBuffer ** buffer)
{
  gpointer queue_item = NULL;
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);
  GstBuffer *buf = NULL;
  GstMemory *mem;
  gsize size = rossrc->payload_size;

  /* get item from queue */
  while (true) {
    queue_item = g_async_queue_timeout_pop (rossrc->queue, G_USEC_PER_SEC / rossrc->freq_rate);
    if (queue_item) {
      silent_debug (rossrc, "queue_item exists!!!\n");
      break;
    }
    /** @todo Return EOF or error */
  };

  /** Use the pre-allocated memory data instead of copying memory space */
  buf = gst_buffer_new ();
  mem = gst_memory_new_wrapped (GST_MEMORY_FLAG_NO_SHARE, queue_item, size,
          0, size, queue_item, g_free);
  gst_buffer_append_memory (buf, mem);
  *buffer = buf;

  silent_debug (rossrc, "Buffer of TensorRosSrc is pushed! (queue size: %d)\n",
    g_async_queue_length (rossrc->queue));

  return GST_FLOW_OK;
}

 /**
 * @brief Initialize the tensor_ros_src plugin
 */
static gboolean
gst_tensor_ros_src_plugin_init (GstPlugin * rossrc)
{
  GST_DEBUG_CATEGORY_INIT (gst_tensor_ros_src_debug, "tensor_ros_src",
      0, "Source element to get the topic data from ROS node");

  return gst_element_register (rossrc, "tensor_ros_src", GST_RANK_NONE,
      GST_TYPE_TENSOR_ROS_SRC);
}

#ifndef PACKAGE
#define PACKAGE "nnstreamer-ros"
#endif

/**
 * @brief Macro to define the entry point of the plugin.
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    tensor_ros_src,
    "Source element to get the topic data from ROS node",
    gst_tensor_ros_src_plugin_init,
    NNS_VERSION,
    "LGPL",
    "nnstreamer-ros",
    "https://github.com/nnsuite/nnstreamer-ros"
)
