/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * Copyright (C) 2019 Sangjung Woo <sangjung.woo@samsung.com>
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *
 * @file        tensor_ros_src.c
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
/**
 * SECTION:element-tensor_ros_src
 *
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * tensor_ros_src and tensor_ros2_src:
 *    Source element to publish tensor stream from deginated Ros topic
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifdef G_OS_WIN32
#include <process.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>
#include <string.h>

#include "nns_ros_subscriber.h"
#include "tensor_ros_src.h"
//#include "tensor_ros_listener.hpp"

#ifdef WITH_ROS1
#define NNS_NAME_TENSOR_ROS_SRC  tensor_ros_src
#else   /** WITH_ROS2 */
#define NNS_NAME_TENSOR_ROS_SRC  tensor_ros2_src
#endif  /** WITH_ROS1 */

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

#define DEFAULT_LIVE_MODE       TRUE

/**
 * @brief Flag to print minimized log.
 */
#define DEFAULT_SILENT TRUE

/**
 * @brief Default timeout in seconds to wait for the next message to receive
 */
#define DEFAULT_TIMEOUT 5.0

/**
 * @brief tensor_ros_src properties
 */
enum
{
  PROP_0,
  PROP_SILENT,      /*<< Slient mode for debug */
  PROP_TOPIC,       /*<< ROS topic name to subscribe */
  PROP_RATE,        /*<< Frequency rate to check the published topic */
  PROP_TIMEOUT,     /*<< Timeout in seconds waiting for the next message to receive */
};

/**
 * @brief Template for ros source pad
 */
static GstStaticPadTemplate src_pad_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_TENSOR_CAP_DEFAULT "; " GST_TENSORS_CAP_DEFAULT));

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
 *  @brief Set the live opreation mode for live source
 */
static void
set_live_mode (GstBaseSrc * src, gboolean mode)
{
  if (gst_base_src_is_live (src) == mode)
    return;

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

  /** GObject methods */
  gobject_class->set_property = gst_tensor_ros_src_set_property;
  gobject_class->get_property = gst_tensor_ros_src_get_property;
  gobject_class->dispose = gst_tensor_ros_src_dispose;

  /** GstBaseSrc method */
  gstbasesrc_class->start = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_start);
  gstbasesrc_class->stop = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_stop);
  gstbasesrc_class->set_caps = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_set_caps);
  gstbasesrc_class->get_caps = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_get_caps);
  gstbasesrc_class->fixate = GST_DEBUG_FUNCPTR (gst_tensor_ros_src_fixate);

  /** GstPushSrc method */
  gstpushsrc_class->create = gst_tensor_ros_src_create;

  /** Install properties */
  /**
   * GstTensorRosSrc::silent:
   *
   * The flag to enable/disable debugging messages.
   */
  g_object_class_install_property (gobject_class, PROP_SILENT,
      g_param_spec_boolean ("silent", "Silent", "Produce verbose output",
          FALSE, G_PARAM_READWRITE));

  /**
   * GstTensorRosSrc::topic:
   *
   * The topic name that this src element subscribes.
   */
  g_object_class_install_property (gobject_class, PROP_TOPIC,
      g_param_spec_string ("topic", "Topic",
          "ROS Topic Name for subscription", "",
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_RATE,
      g_param_spec_double ("rate", "Rate",
          "The desired rate to run at in Hz (Default: 1).",
          (gdouble) 1, (gdouble) G_USEC_PER_SEC, (gdouble) 1,
          G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_TIMEOUT,
      g_param_spec_double ("timeout", "Timeout",
          "Timeout in seconds waiting for the next message to receive (Default: 5).",
          (gdouble) 1, (gdouble) 600, (gdouble) DEFAULT_TIMEOUT,
          G_PARAM_READWRITE));

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
  rossrc->rate = (gdouble)  1;
  rossrc->timeout = (gdouble) DEFAULT_TIMEOUT;
  rossrc->queue = NULL;
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
    nns_ros_subscriber_put_queue (rossrc->nns_ros_bind_instance);

  if (rossrc->topic_name)
    g_free (rossrc->topic_name);
  rossrc->configured = FALSE;

  nns_ros_subscriber_fianlize (rossrc->nns_ros_bind_instance);

  G_OBJECT_CLASS (parent_class)->dispose (object);
}

/**
 * @brief Start handler
 */
static gboolean
gst_tensor_ros_src_start (GstBaseSrc * src)
{
  static const gchar format_node_name_pid[] = "PID_%d_%s";
  GstTensorRosSrc *rossrc;
  gchar *name_node;
  gint32 pid;

  rossrc = GST_TENSOR_ROS_SRC (src);
  g_return_val_if_fail (GST_IS_TENSOR_ROS_SRC (rossrc), FALSE);
  if ((rossrc->topic_name == NULL) || (strlen (rossrc->topic_name) == 0)) {
    g_critical (
        "ERROR: Failed to start %s: topic to subscribe should be given.",
        GST_ELEMENT_NAME (GST_ELEMENT (src)));
    return FALSE;
  }

  pid = getpid ();
  name_node = g_strdup_printf (format_node_name_pid, pid,
      GST_ELEMENT_NAME (GST_ELEMENT (src)));

  rossrc->nns_ros_bind_instance = nns_ros_subscriber_init (name_node,
      rossrc->topic_name, rossrc->rate, rossrc->timeout, &rossrc->configs,
      FALSE);
  g_free (name_node);

  g_return_val_if_fail (rossrc->nns_ros_bind_instance != NULL, FALSE);

  rossrc->queue = nns_ros_subscriber_get_queue (rossrc->nns_ros_bind_instance);

  rossrc->configured = TRUE;
  gst_base_src_start_complete (src, GST_FLOW_OK);

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

  return TRUE;
}

/**
 * @brief Get caps handler
 */
static GstCaps *
gst_tensor_ros_src_get_caps (GstBaseSrc * src, GstCaps * filter)
{
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

  return caps;
}

/**
 * @brief Set caps handler
 */
static gboolean
gst_tensor_ros_src_set_caps (GstBaseSrc * src, GstCaps * caps)
{
  return gst_pad_set_caps (src->srcpad, caps);
}

static GstCaps *
gst_tensor_ros_src_fixate (GstBaseSrc * src, GstCaps * caps)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);
  GstCaps *fixated_caps;
  GstCaps *update_caps;
  GstCaps *ret_caps;

  if (rossrc->configs.info.num_tensors == 1) {
    GstTensorConfig tensor_config;

    gst_tensor_info_copy (&tensor_config.info,
        &(rossrc->configs.info.info[0]));
    tensor_config.rate_n = rossrc->configs.rate_n;
    tensor_config.rate_d = rossrc->configs.rate_d;
    fixated_caps = gst_tensor_caps_from_config (&tensor_config);
    gst_tensor_info_free (&tensor_config.info);
  } else {
    fixated_caps = gst_tensors_caps_from_config (&rossrc->configs);
  }

  if (fixated_caps == NULL) {
    GST_ERROR_OBJECT (rossrc, "Error creating fixated caps from config.");
    return NULL;
  }

  update_caps = gst_caps_intersect (caps, fixated_caps);
  if (!update_caps) {
    GST_ERROR_OBJECT (rossrc,
        "No intersection while fixating caps of the element.");
    ret_caps = NULL;
  } else {
    ret_caps =gst_caps_fixate (update_caps);
  }

  gst_caps_unref (caps);
  gst_caps_unref (fixated_caps);

  return ret_caps;
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
      silent_debug (rossrc, "Silent Mode: %s\n",
          rossrc->silent ? "true" : "false");
      break;

    case PROP_TOPIC:
      rossrc->topic_name = g_value_dup_string (value);
      silent_debug (rossrc, "topic name: %s\n", rossrc->topic_name);
      break;

    case PROP_RATE:
      rossrc->rate = g_value_get_double (value);
      silent_debug (rossrc, "Rate in Hz: %lf\n", rossrc->rate);
      break;

    case PROP_TIMEOUT:
      rossrc->timeout = g_value_get_double (value);
      silent_debug (rossrc, "Timeout in seconds: %lf\n", rossrc->timeout);
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

    case PROP_RATE:
      g_value_set_double (value, rossrc->rate);
      break;

    case PROP_TIMEOUT:
      g_value_set_double (value, rossrc->timeout);
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
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);
  const guint64 timeout = rossrc->timeout * G_USEC_PER_SEC;
  const guint num_tensors = rossrc->configs.info.num_tensors;
  GstBuffer *buf = NULL;
  GstMemory *mem = NULL;
  gpointer data_tensors = NULL;
  gsize size_tensors = 0;
  guint i;

  for (i = 0; i < num_tensors; ++i) {
    gsize size = gst_tensor_info_get_size (&(rossrc->configs.info.info[i]));

    size_tensors += size;
  }

  data_tensors = g_async_queue_timeout_pop (rossrc->queue, timeout);
  if (!data_tensors)
    return GST_FLOW_EOS;

  buf = gst_buffer_new ();
  mem = gst_memory_new_wrapped (GST_MEMORY_FLAG_NO_SHARE, data_tensors,
      size_tensors, 0, size_tensors, data_tensors, g_free);
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
  GST_DEBUG_CATEGORY_INIT (gst_tensor_ros_src_debug,
      G_STRINGIFY (NNS_NAME_TENSOR_ROS_SRC), 0,
      "Source element to get the topic data from ROS node");

  return gst_element_register (rossrc, G_STRINGIFY (NNS_NAME_TENSOR_ROS_SRC),
      GST_RANK_NONE, GST_TYPE_TENSOR_ROS_SRC);
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
    NNS_NAME_TENSOR_ROS_SRC,
    "Source element to get the topic data from ROS node",
    gst_tensor_ros_src_plugin_init,
    NNS_VERSION,
    "LGPL",
    "nnstreamer-ros",
    "https://github.com/nnstreamer/nnstreamer-ros"
)
