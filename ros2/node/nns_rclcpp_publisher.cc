/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * @file        nns_rclcpp_publisher.cc
 * @date        10/20/2020
 * @brief       A helper class for publishing ROS2 (i.e., rclcpp) topic
 *              via NNStreamer plugins
 * @see         https://github.com/nnstreamer/nnstreamer-ros
 * @author      Wook Song <wook16.song@samsung.com>
 * @bug         No known bugs except for NYI items
 *
 * This class bridges between the NNStreamer (C) and ROS2 frameworks (ROSCPP/C++)
 * by implementing the NnsRosPublisher class.
 *
 */
#include <glib.h>
#include <glib/gstdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2/writers/sequential_writer.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/writer.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "nns_ros_publisher.h"
#include "nns_ros2_bridge/msg/tensors.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

using namespace std_msgs::msg;

class NnsRclCppPublisher
    : public NnsRosPublisher<MultiArrayLayout, size_t>
{
public:
  NnsRclCppPublisher (const char *node_name, const char *topic_name,
      gboolean is_dummy);
  ~NnsRclCppPublisher ();

  gboolean publish (const guint num_tensors,
      const GstTensorMemory *tensors_mem, void *bag) final;
  void setRosBagWriter (
      std::unique_ptr<rosbag2::writers::SequentialWriter> writer);
  std::unique_ptr<rosbag2::writers::SequentialWriter> getRosBagWriter ();

private:
  static const std::string node_namespace_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nns_ros2_bridge::msg::Tensors>::SharedPtr publisher_;
  std::unique_ptr<rosbag2::writers::SequentialWriter> rb2_sq_writer_;
};

const std::string NnsRclCppPublisher::node_namespace_ = "tensor_ros2_sink";

template <>
std::string NnsRosPublisher<MultiArrayLayout, size_t>::str_nns_helper_name =
    "nns_rclcpp_publisher";

NnsRclCppPublisher::NnsRclCppPublisher (const char *node_name,
    const char *topic_name, gboolean is_dummy)
    : NnsRosPublisher<MultiArrayLayout, size_t> (node_name, topic_name,
        is_dummy)
{
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
      NnsRclCppPublisher::node_namespace_);
  } catch (rclcpp::exceptions::RCLErrorBase &e) {
    g_critical ("%s: Failed to create a rclcpp::Node instance: %s",
        (this->str_nns_helper_name).c_str (), e.message.c_str ());
    throw ROS2_FAILED_TO_CREATE_NODE;
  }

  try {
    this->publisher_ =
      this->node_->create_publisher<nns_ros2_bridge::msg::Tensors> (
          topic_name, default_q_size);
  } catch (rclcpp::exceptions::RCLErrorBase &e) {
    g_critical ("%s: Failed to create a rclcpp::Node instance: %s",
        (this->str_nns_helper_name).c_str (), e.message.c_str ());
    throw ROS2_FAILED_TO_CREATE_PUBLISHER;
  }
}

NnsRclCppPublisher::~NnsRclCppPublisher ()
{
  bool ret = rclcpp::shutdown();

  if (!ret)
    g_critical ("%s: Failed to shutdown the given context",
        (this->str_nns_helper_name).c_str ());
}

/**
 * @brief A method for publishing a ROS topic that contains the tensors passed by the NNStreamer framework
 * @param[in] num_tensors The number of tensors included in tensors_mem (for the verification purpose)
 * @param[in] tensors_mem The pointer of containers consists of information and raw data of tensors
 * @return TRUE if the configuration is successfully done
 */
gboolean NnsRclCppPublisher::publish (const guint num_tensors,
    const GstTensorMemory *tensors_mem, void *bag __attribute__((unused)))
{
  nns_ros2_bridge::msg::Tensors tensors_msg;
  std::unique_ptr<rosbag2::writers::SequentialWriter> sq_writer;
  gboolean ret = TRUE;
  rcutils_ret_t rb2_ret;

  g_return_val_if_fail (this->ready_to_pub, FALSE);
  g_return_val_if_fail (num_tensors == this->num_of_tensors_pub, FALSE);

  for (guint i = 0; i < this->num_of_tensors_pub; ++i) {
    UInt8MultiArray each_tensor;
    uint8_t *src_data = (uint8_t *) tensors_mem[i].data;

    each_tensor.layout = this->topic_layouts_pub[i];
    each_tensor.data.assign (src_data, src_data + tensors_mem[i].size);

    tensors_msg.tensors.push_back (each_tensor);
  }

  if (!this->is_dummy) {
    g_return_val_if_fail (rclcpp::ok(), FALSE);
    this->publisher_->publish (tensors_msg);
    rclcpp::spin_some(this->node_);
  }

  sq_writer = this->getRosBagWriter ();
  if (sq_writer) {
    rcutils_allocator_t allocator = rcutils_get_default_allocator ();
    rcutils_uint8_array_t  serialized_message =
        rmw_get_zero_initialized_serialized_message ();
    auto typesupport_cpp =
        rosidl_typesupport_cpp::get_message_type_support_handle<
            nns_ros2_bridge::msg::Tensors
        > ();
    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage> ();

    rb2_ret =rmw_serialized_message_init (&serialized_message, 0, &allocator);
    if (rb2_ret != RMW_RET_OK) {
      g_critical ("%s: rosbag2: Failed to initialize rmw_serialized_message\n",
          (this->str_nns_helper_name).c_str ());
      ret = FALSE;
      goto restore_sq_writer;
    }

    rb2_ret = rmw_serialize (&tensors_msg, typesupport_cpp,
        &serialized_message);
    if (rb2_ret != RMW_RET_OK) {
      g_critical ("%s: rosbag2: Failed to serialize a message of the nns_ros2_bridge::msg::Tensors type\n",
          (this->str_nns_helper_name).c_str ());
      ret = FALSE;
      goto restore_sq_writer;
    }
    message->serialized_data =
        std::make_shared<rcutils_uint8_array_t> (serialized_message);
    message->topic_name = this->getPubTopicName ();
    rb2_ret = rcutils_system_time_now (&message->time_stamp);
    if (rb2_ret != RCUTILS_RET_OK) {
      g_critical ("%s: rosbag2: Failed to obtatin the current time\n",
          (this->str_nns_helper_name).c_str ());
      ret = FALSE;
      goto restore_sq_writer;
    } else {
      sq_writer->write (message);
    }

restore_sq_writer:
    this->setRosBagWriter (std::move (sq_writer));
  }

  return ret;
}

/**
 * @brief A method to move the ownership for a given argument of
 *        the std::unique_ptr<rosbag2::writers::SequentialWriter> type to the
 *        member variable rb2_sq_writer_ of this class
 * @param[in] writer a unique_ptr of rosbag2::writers::SequentialWriter to move
 */
void
NnsRclCppPublisher::setRosBagWriter (
      std::unique_ptr<rosbag2::writers::SequentialWriter> writer)
{
  this->rb2_sq_writer_ = std::move (writer);
}

/**
 * @brief A method to get the unique_ptr of rosbag2::writers::SequentialWriter
 *        which this class holds
 * @return the unique_ptr moved from this->rb2_sq_writer_. If this->rb2_sq_writer_
 *         is not assigned, nullptr is returned.
  */
std::unique_ptr<rosbag2::writers::SequentialWriter>
NnsRclCppPublisher::getRosBagWriter ()
{
  std::unique_ptr<rosbag2::writers::SequentialWriter> writer;

  if (!this->rb2_sq_writer_)
    return nullptr;

  writer = std::move (this->rb2_sq_writer_);

  return writer;
}

/**
 * C functions for NNStreamer plugins that want to publish or subscribe
 * ROS2 topics. nns_ros_publisher_init() should be invoked before configuring the
 * tensors information to publish or subscribe. Each NNStreamer plugin which
 * uses this class should be holding the instance of NnsRclCppPublisher (i.e.,
 * the void type pointer what the nns_ros_publisher_init() function returns).
 */
void *
nns_ros_publisher_init (const char *node_name, const char *topic_name,
    gboolean is_dummy)
{
  try {
    return new NnsRclCppPublisher (node_name, topic_name, is_dummy);
  } catch (const err_code e) {
    return nullptr;
  }
}

void
nns_ros_publisher_finalize (void *instance)
{
  NnsRclCppPublisher *nrp_instance = (NnsRclCppPublisher *) instance;
  if (nrp_instance != nullptr)
    delete nrp_instance;
}

gboolean
nns_ros_publisher_publish (void *instance, const guint num_tensors,
    const GstTensorMemory *tensors_mem, void *bag)
{
  NnsRclCppPublisher *nrp_instance = (NnsRclCppPublisher *) instance;

  return nrp_instance->publish (num_tensors, tensors_mem, bag);
}

gboolean
nns_ros_publisher_set_pub_topic (void *instance, const GstTensorsConfig *conf)
{
  NnsRclCppPublisher *nrp_instance = (NnsRclCppPublisher *) instance;

  return nrp_instance->setPubTopicInfo<MultiArrayDimension> (conf);
}

const gchar *
nns_ros_publisher_get_pub_topic_name (void *instance)
{
  NnsRclCppPublisher *nrp_instance = (NnsRclCppPublisher *) instance;

  return nrp_instance->getPubTopicName();
}

void *
nns_ros_publisher_open_writable_bag (void *instance, const char *name)
{
  NnsRclCppPublisher *nrp_instance = (NnsRclCppPublisher *) instance;
  static const std::string topic_name =
      nns_ros_publisher_get_pub_topic_name (instance);
  /**
   * Variables for creationg of a rosbag2 writer
   * @todo Do we need to make them configurable?
   */
  static const std::string storage_id = "sqlite3";
  static const std::string serialization_format = "cdr";
  static const std::string topic_type = "nns_ros2_bridge/msg/Tensors";
  /** Bag file splitting is not used */
  const uint64_t max_bagfile_size = 0;
  const mode_t mode = 0755;

  std::unique_ptr<rosbag2::writers::SequentialWriter> sq_writer =
      std::make_unique <rosbag2::writers::SequentialWriter> ();

  rosbag2_storage::TopicMetadata rb2_topic_meta;
  rosbag2::ConverterOptions rb2_converter_options;
  rosbag2::StorageOptions rb2_storage_options;
  rosbag2::writers::SequentialWriter *sq_writer_ptr;
  char *path_rosbag;

  if (name == NULL || name[0] == '\0') {
    gchar *cur_dir = g_get_current_dir ();

    path_rosbag = g_build_filename (cur_dir, topic_name.c_str (), NULL);
    g_free (cur_dir);
  } else {
    path_rosbag = g_strdup (name);
  }

  /**
   * std::string uri in the rosbag2::StorageOptions structure should be an
   * existing directory pathname. In Eloquent, if uri is given as a URI format,
   * the 'Failed to create bag' error occurs. Moreover, if there is a file
   * whose name is the same as the file that the rosbag2 module tries to create,
   * the error would occur. IMO, the features provided by the rosbag2 APIs do
   * not look that mature. To simplify those incomprehensible and complex
   * conditions, let's assume that the given path does not exist and a directory
   * will be created by the given pathname here.
   */
  if (g_file_test (path_rosbag, G_FILE_TEST_EXISTS) ||
      (g_mkdir (path_rosbag, mode) == -1)) {
    g_free (path_rosbag);

    return nullptr;
  }

  rb2_storage_options.max_bagfile_size = max_bagfile_size;
  rb2_storage_options.storage_id = storage_id;
  rb2_storage_options.uri = g_strdup (path_rosbag);
  g_free (path_rosbag);

  rb2_topic_meta.name = topic_name;
  rb2_topic_meta.type = topic_type;
  rb2_topic_meta.serialization_format = serialization_format;

  rb2_converter_options.input_serialization_format = serialization_format;
  rb2_converter_options.output_serialization_format = serialization_format;

  sq_writer->open (rb2_storage_options, rb2_converter_options);
  sq_writer->create_topic (rb2_topic_meta);
  sq_writer_ptr = sq_writer.get ();
  nrp_instance->setRosBagWriter (std::move (sq_writer));

  return sq_writer_ptr;
}

void
nns_ros_publisher_close_bag (void *instance, void *bag)
{
  NnsRclCppPublisher *nrp_instance = (NnsRclCppPublisher *) instance;
  std::unique_ptr<rosbag2::writers::SequentialWriter> sq_writer;

  if(instance == NULL)
    return;

  sq_writer = nrp_instance->getRosBagWriter ();

  if ((bag) && (sq_writer.get () == bag))
    sq_writer.release ();
}
