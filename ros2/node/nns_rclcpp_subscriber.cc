/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * @file        nns_rclcpp_subscriber.cc
 * @date        10/28/2020
 * @brief       A helper class for subscribing ROS2 (i.e., rclcpp) topic
 *              via NNStreamer plugins
 * @see         https://github.com/nnstreamer/nnstreamer-ros
 * @author      Wook Song <wook16.song@samsung.com>
 * @bug         No known bugs except for NYI items
 *
 * This class bridges between the NNStreamer (C) and ROS2 frameworks (RCLCPP/C++)
 * by implementing the NnsRosSubscriber class.
 *
 */
#ifdef G_OS_WIN32
#include <process.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif
#include <glib.h>
#include <glib/gstdio.h>
#include <nnstreamer/tensor_typedef.h>

#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/readers/sequential_reader.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "nns_ros_subscriber.h"
#include "nns_ros2_bridge/msg/tensors.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

using namespace std_msgs::msg;
using std::placeholders::_1;

class NnsRclCppSubscriber
    : public NnsRosSubscriber<nns_ros2_bridge::msg::Tensors>
{
public:
  NnsRclCppSubscriber (const gchar *node_name, const char *topic_name,
      const gdouble rate, const gdouble timeout, GstTensorsConfig *configs,
      gboolean is_dummy);
  ~NnsRclCppSubscriber ();

  void setRosBagReader (
      std::unique_ptr<rosbag2::readers::SequentialReader> reader);
  std::unique_ptr<rosbag2::readers::SequentialReader> getRosBagReader ();
  void fetchDataFromBag ();
  void subCallback (const nns_ros2_bridge::msg::Tensors::SharedPtr msg) final;
  void configure (const nns_ros2_bridge::msg::Tensors::SharedPtr msg) final;
  void finalize () final;

private:
  NnsRclCppSubscriber () {};
  void loop ();

  static const std::string node_namespace_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nns_ros2_bridge::msg::Tensors>::SharedPtr subscriber_;
  std::unique_ptr<rosbag2::readers::SequentialReader> rb2_sq_reader_;
};

template <>
std::string NnsRosSubscriber<nns_ros2_bridge::msg::Tensors>::str_nns_helper_name =
    "nns_rclcpp_subscriber";

const std::string NnsRclCppSubscriber::node_namespace_ = "tensor_ros2_src";

/**
 * @brief	The only public constructor of this class
 * @param[in] node_name The name of the ROS node under /tensor_ros_sink
 * @param[in] topic_name The name of topic what this class concerns
 * @param[in] rate The subscribing rate in Hz
 * @param[in] timeout The value indicating the timeout in seconds to wait for the next message to receive
 * @param[out] configs The pointer to the tensor_ros_src's GstTensorsConfig to set
 * @param[in] is_dummy If TRUE, create the instance without roscore connection
 * @return None
 */
NnsRclCppSubscriber::NnsRclCppSubscriber (const char *node_name,
    const char *topic_name, const gdouble rate, const gdouble timeout,
    GstTensorsConfig *configs, gboolean is_dummy)
    : NnsRosSubscriber<nns_ros2_bridge::msg::Tensors> (node_name, topic_name,
        rate, timeout, configs, is_dummy)
{
  rclcpp::Rate ros_rate (this->rate);
  std::chrono::_V2::system_clock::time_point st;
  /* RCLCPP initialization requires command-line arguments */
  int dummy_argc = 0;
  char **dummy_argv = NULL;

  try {
    rclcpp::init (dummy_argc, dummy_argv);
  } catch (rclcpp::ContextAlreadyInitialized &e) {
    g_warning ("%s: The given context has been already initialized",
        (this->str_nns_helper_name).c_str ());
  }

  try {
    this->node_ = rclcpp::Node::make_shared (node_name,
        NnsRclCppSubscriber::node_namespace_);
  } catch (rclcpp::exceptions::RCLErrorBase &e) {
    g_critical ("%s: Failed to create a rclcpp::Node instance: %s",
        (this->str_nns_helper_name).c_str (), e.message.c_str ());
    throw ROS2_FAILED_TO_CREATE_NODE;
  }

  if (this->is_dummy) {
    this->subscriber_ = nullptr;
  } else {
    try {
        this->subscriber_ =
            this->node_->create_subscription<nns_ros2_bridge::msg::Tensors> (
                topic_name, 10,
                std::bind (&NnsRclCppSubscriber::configure, this,
                    std::placeholders::_1)
            );
    } catch (rclcpp::exceptions::RCLErrorBase &e) {
      g_critical ("%s: Failed to create a subscription for GstCap configuration: %s",
        (this->str_nns_helper_name).c_str (), e.message.c_str ());
      throw ROS_FAILED_TO_CREATE_SUBSCRIPTION;
    }

    st = std::chrono::system_clock::now ();
    while (rclcpp::ok ()) {
      std::chrono::duration<double> diff;
      rclcpp::spin_some (this->node_);
      ros_rate.sleep ();

      {
        std::unique_lock<std::mutex> lk (this->g_m);
        if (this->configured)
          break;
      }

      diff = std::chrono::system_clock::now () - st;
      if (diff > this->timeout) {
        g_critical ("%s: Failed to configure GstCaps from the given topic",
          (this->str_nns_helper_name).c_str ());
        throw ROS_SUBSCRIPTION_TIMED_OUT;
      }
    }

    this->subscriber_.reset ();

    try {
      this->subscriber_ =
          this->node_->create_subscription<nns_ros2_bridge::msg::Tensors> (
              topic_name, 10,
              std::bind (&NnsRclCppSubscriber::subCallback, this,
                  std::placeholders::_1)
              );
    } catch (rclcpp::exceptions::RCLErrorBase &e) {
      g_critical ("%s: Failed to create a subscription for the given topic: %s",
          (this->str_nns_helper_name).c_str (), e.message.c_str ());
      throw ROS_FAILED_TO_CREATE_SUBSCRIPTION;
    }
  }

  this->looper = std::make_shared <std::thread> (&NnsRclCppSubscriber::loop,
      this);
  {
    std::lock_guard<std::mutex> lk (this->looper_m);
    this->is_looper_ready = true;
  }
  this->looper_cv.notify_all ();
}

NnsRclCppSubscriber::~NnsRclCppSubscriber ()
{
  if (!rclcpp::shutdown())
    g_critical ("%s: Failed to shutdown the given context",
        (this->str_nns_helper_name).c_str ());
}

void
NnsRclCppSubscriber::finalize ()
{
  {
    std::unique_lock<std::mutex> lk (this->g_m);
    this->is_looper_ready = false;
    this->configured = false;
  }
  this->looper->join ();
  if (!this->gasyncq)
    g_async_queue_unref (this->gasyncq);
}

void
NnsRclCppSubscriber::configure (
  const nns_ros2_bridge::msg::Tensors::SharedPtr msg)
{
  guint num_tensors;

  {
    std::unique_lock<std::mutex> lk (this->g_m);
    if (this->configured)
      return;
  }

  num_tensors = static_cast<guint> (msg->tensors.size ());

  this->configs->info.num_tensors = num_tensors;
  this->configs->rate_n = this->rate * 10000;
  this->configs->rate_d = 10000;

  for (guint i = 0; i < num_tensors; ++i) {
    GstTensorInfo *info = &(this->configs->info.info[i]);
    std_msgs::msg::MultiArrayLayout layout = msg->tensors[i].layout;

    for (guint j = 0; j < NNS_TENSOR_RANK_LIMIT; ++j) {
      guint nns_dim = NNS_TENSOR_RANK_LIMIT - 1 - j;

      info->dimension[nns_dim] = static_cast<guint> (layout.dim[j].size);
      info->type = gst_tensor_get_type (layout.dim[j].label.c_str ());
    }
  }

  std::unique_lock<std::mutex> lk (this->g_m);
  this->configured = true;
}

void
NnsRclCppSubscriber::subCallback (
  const nns_ros2_bridge::msg::Tensors::SharedPtr msg)
{
  guint num_tensors = this->configs->info.num_tensors;
  guint size_tensors = 0;

  for (guint i = 0; i < num_tensors; ++i) {
    gsize size = gst_tensor_info_get_size (&(this->configs->info.info[i]));

    size_tensors += size;
  }

  gpointer data = g_malloc0 (size_tensors);

  for (guint i = 0; i < num_tensors; ++i) {
    uint8_t *off = (uint8_t *) data;
    gsize size = gst_tensor_info_get_size (&(this->configs->info.info[i]));

    std::memcpy (off, msg->tensors.at (i).data.data (), size);
    off += size;
  }

  g_async_queue_push (this->gasyncq, data);
}

/**
 * @brief A method to move the ownership for a given argument of
 *        the std::unique_ptr<rosbag2::readers::SequentialReader> type to the
 *        member variable rb2_sq_reader_ of this class
 * @param[in] writer a unique_ptr of rosbag2::readers::SequentialReader to move
 */
void
NnsRclCppSubscriber::setRosBagReader (
      std::unique_ptr<rosbag2::readers::SequentialReader> reader)
{
  this->rb2_sq_reader_ = std::move (reader);
}

/**
 * @brief A method to get the unique_ptr of rosbag2::readers::SequentialReader
 *        which this class holds
 * @return the unique_ptr moved from this->rb2_sq_reader_. If this->rb2_sq_writer_
 *         is not assigned, nullptr is returned.
  */
std::unique_ptr<rosbag2::readers::SequentialReader>
NnsRclCppSubscriber::getRosBagReader ()
{
  std::unique_ptr<rosbag2::readers::SequentialReader> reader;

  if (!this->rb2_sq_reader_)
    return nullptr;

  reader = std::move (this->rb2_sq_reader_);

  return reader;
}

void NnsRclCppSubscriber::loop ()
{
  rclcpp::Rate ros_rate (this->rate);
  std::unique_lock<std::mutex> lk (this->looper_m);

  this->looper_cv.wait (lk,
      [&] () -> gboolean
      {
        return this->is_looper_ready;
      }
  );

  while (true) {
    if (!this->is_dummy)
      rclcpp::spin_some (this->node_);
    else
      this->fetchDataFromBag ();
    ros_rate.sleep ();
    {
      std::unique_lock<std::mutex> lk (this->g_m);
      if (!this->is_looper_ready)
        break;
    }
  }
}

void NnsRclCppSubscriber::fetchDataFromBag ()
{
  if (((!this->rb2_sq_reader_)) || (!this->rb2_sq_reader_->has_next ()))
    return;

  auto serialized_message = this->rb2_sq_reader_->read_next();
  // deserialization and conversion to ros message
  nns_ros2_bridge::msg::Tensors::SharedPtr msg =
      std::make_shared<nns_ros2_bridge::msg::Tensors> ();
  auto ros_message = std::make_shared<rosbag2_introspection_message_t>();
  auto type_support = rosidl_typesupport_cpp::get_message_type_support_handle<
      nns_ros2_bridge::msg::Tensors
      > ();

  rosbag2::SerializationFormatConverterFactory factory;
  std::unique_ptr<rosbag2::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;

  ros_message->time_stamp = 0;
  ros_message->message = nullptr;
  ros_message->allocator = rcutils_get_default_allocator();
  ros_message->message = msg.get();
  cdr_deserializer_ = factory.load_deserializer("cdr");
  cdr_deserializer_->deserialize(serialized_message, type_support, ros_message);
  this->configure (msg);
  this->subCallback (msg);
}

void *
nns_ros_subscriber_open_readable_bag (void *instance, const char *path)
{
  NnsRclCppSubscriber *nrp_instance = (NnsRclCppSubscriber *) instance;
  static const std::string storage_id = "sqlite3";
  static const std::string serialization_format = "cdr";
  static const std::string topic_type = "nns_ros2_bridge/msg/Tensors";
  const uint64_t max_bagfile_size = 0;
  std::unique_ptr<rosbag2::readers::SequentialReader> sq_reader =
      std::make_unique <rosbag2::readers::SequentialReader> ();
  rosbag2_storage::TopicMetadata rb2_topic_meta;
  rosbag2::ConverterOptions rb2_converter_options;
  rosbag2::StorageOptions rb2_storage_options;
  rosbag2::readers::SequentialReader *sq_reader_ptr;
  char *path_rosbag = g_build_filename (path, NULL);

  /**
   * std::string uri in the rosbag2::StorageOptions structure should be an
   * existing file pathname. In Eloquent, if uri is given as a URI format,
   * the 'Failed to create bag' error occurs.
   */
  if (!g_file_test (path_rosbag, G_FILE_TEST_EXISTS)) {
    g_critical ("ERROR: The given path of the rosbag2 file, \"%s\", is not valid.\n",
        path_rosbag);
    g_free (path_rosbag);

    return nullptr;
  }

  rb2_storage_options.max_bagfile_size = max_bagfile_size;
  rb2_storage_options.storage_id = storage_id;
  rb2_storage_options.uri = g_strdup (path_rosbag);
  g_free (path_rosbag);

  rb2_topic_meta.name = nrp_instance->getSubTopicName ();
  rb2_topic_meta.type = topic_type;
  rb2_topic_meta.serialization_format = serialization_format;

  rb2_converter_options.input_serialization_format = serialization_format;
  rb2_converter_options.output_serialization_format = serialization_format;

  sq_reader->open (rb2_storage_options, rb2_converter_options);
  sq_reader_ptr = sq_reader.get ();
  nrp_instance->setRosBagReader (std::move (sq_reader));
  nrp_instance->fetchDataFromBag ();

  return sq_reader_ptr;
}

/**
 * C functions for NNStreamer plugins that want to publish or subscribe
 * ROS2 topics. nns_ros_subscriber_init() should be invoked before configuring the
 * tensors information to publish or subscribe. Each NNStreamer plugin which
 * uses this class should be holding the instance of NnsRclCppSubscriber (i.e.,
 * the void type pointer what the nns_ros_subscriber_init() function returns).
 */
void *nns_ros_subscriber_init (const gchar *node_name, const char *topic_name,
      const gdouble rate, const gdouble timeout, GstTensorsConfig *configs,
      gboolean is_dummy)
{
  try {
    return new NnsRclCppSubscriber (node_name, topic_name, rate, timeout,
        configs, is_dummy);
  } catch (const err_code e) {
    return nullptr;
  }
}

void nns_ros_subscriber_fianlize (void *instance)
{
  NnsRclCppSubscriber *nrs_instance = (NnsRclCppSubscriber *) instance;

  nrs_instance->finalize ();

  if (nrs_instance)
    delete nrs_instance;
}

GAsyncQueue *nns_ros_subscriber_get_queue (void *instance)
{
  NnsRclCppSubscriber *nrs_instance = (NnsRclCppSubscriber *) instance;

  return nrs_instance->getQueue ();
}

void nns_ros_subscriber_put_queue (void *instance)
{
  NnsRclCppSubscriber *nrs_instance = (NnsRclCppSubscriber *) instance;

  nrs_instance->putQueue ();
}
