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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
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

private:
  static const std::string node_namespace_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nns_ros2_bridge::msg::Tensors>::SharedPtr publisher_;
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
 * @brief	A method for publishing a ROS topic that contains the tensors passed by the NNStreamer framework
 * @param[in] num_tensors The number of tensors included in tensors_mem (for the verification purpose)
 * @param[in] tensors_mem The pointer of containers consists of information and raw data of tensors
 * @return TRUE if the configuration is successfully done
 */
gboolean NnsRclCppPublisher::publish (const guint num_tensors,
    const GstTensorMemory *tensors_mem, void *bag __attribute__((unused)))
{
  nns_ros2_bridge::msg::Tensors tensors_msg;

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

  return TRUE;
}

/**
 * C functions for NNStreamer plugins that want to publish or subscribe
 * ROS topics. nns_ros_publisher_init() should be invoked before configuring the
 * tensors information to publish or subscribe. Each NNStreamer plugin which
 * uses this class should be holding the instance of NnsRclCppPublisher (i.e., the
 * void type pointer what the nns_ros_publisher_init() function returns).
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
nns_ros_publisher_open_writable_bag (void * instance __attribute__((unused)), const char *name __attribute__((unused)))
{
  return nullptr;
}

void
nns_ros_publisher_close_bag (void * instance __attribute__((unused)), void *bag __attribute__((unused)))
{
  ;
}
