/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * Copyright (C) 2019 Sangjung Woo <sangjung.woo@samsung.com>
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *
 * @file        nns_ros_subscriber.cc
 * @date        03/06/2019
 * @brief       A helper class for publishing ROS1 (i.e., roscpp) topic
 *              via NNStreamer plugins
 * @see         https://github.com/nnstreamer/nnstreamer-ros
 * @author      Sangjung Woo <sangjung.woo@samsung.com>
 *              Wook Song <wook16.song@samsung.com>
 * @bug         No known bugs except for NYI items
 *
 * This class bridges between the NNStreamer (C) and ROS1 frameworks
 * (ROSCPP/C++) by implementing the NnsRosSubscriber class.
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
#include <chrono>
#include <mutex>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

#include "nns_ros_bridge/Tensors.h"
#include "nns_ros_subscriber.h"

const char BASE_NODE_NAME[] = "tensor_ros_src";

class NnsRosCppSubscriber
    : public NnsRosSubscriber<nns_ros_bridge::Tensors>
{
public:
  NnsRosCppSubscriber (const gchar *node_name, const char *topic_name,
      const gdouble rate, const gdouble timeout, GstTensorsConfig *configs,
      gboolean is_dummy);
  ~NnsRosCppSubscriber ();

  void subCallback (const std::shared_ptr<nns_ros_bridge::Tensors> msg) final;
  void configure (const std::shared_ptr<nns_ros_bridge::Tensors> msg) final;
  void finalize () final;
  bool is_configured () { return this->configured; }

  // @todo: Use smart_ptrs
  void setRosBagReader (rosbag::Bag *bag)  {this->bag_ = bag; }
  rosbag::Bag *getRosBagReader () { return this->bag_; }
  void fetchDataFromBag ();

private:
  NnsRosCppSubscriber () {};
  void loop ();

  // Variables for ROS configuration
  ros::NodeHandle *nh_parent;
  ros::NodeHandle *nh_child;
  ros::Subscriber ros_src_sub;
  void subCallbackInternal (const nns_ros_bridge::Tensors msg);
  void configureInternal (const nns_ros_bridge::Tensors msg);
  // Variables for ROSBAG
  rosbag::Bag *bag_;
};

template <>
std::string NnsRosSubscriber<nns_ros_bridge::Tensors>::str_nns_helper_name =
    "nns_roscpp_subscriber";

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
NnsRosCppSubscriber::NnsRosCppSubscriber (const gchar *node_name,
    const char *topic_name, const gdouble rate, const gdouble timeout,
    GstTensorsConfig *configs, gboolean is_dummy)
    : NnsRosSubscriber<nns_ros_bridge::Tensors> (node_name, topic_name,
        rate, timeout, configs, is_dummy)
{
  std::chrono::_V2::system_clock::time_point st;
  /* ROS initialization requires command-line arguments */
  int dummy_argc = 0;
  char **dummy_argv = NULL;
  ros::Subscriber tmp_sub;

  this->bag_ = NULL;

  if (!is_dummy) {

    if (getenv ("ROS_MASTER_URI") == NULL) throw ROS1_UNDEFINED_ROS_MASTER_URI;

    ros::init (dummy_argc, dummy_argv, this->str_node_name);

    if (!ros::master::check ()) throw ROS1_FAILED_TO_CONNECT_ROSCORE;

    this->nh_parent = new ros::NodeHandle (BASE_NODE_NAME);
    this->nh_child =
        new ros::NodeHandle (*(this->nh_parent), this->str_node_name);

    try {
      tmp_sub = this->nh_child->subscribe (
          this->str_sub_topic_name,
          this->default_ros_queue_size,
          &NnsRosCppSubscriber::configureInternal, this);
    } catch (ros::Exception &e) {
      g_critical ("%s: Failed to create a subscription for the given topic: %s",
          (this->str_nns_helper_name).c_str (), e.what ());
      throw ROS_FAILED_TO_CREATE_SUBSCRIPTION;
    }

    st = std::chrono::system_clock::now ();

    ros::Rate ros_rate (this->rate);
    while (ros::ok ()) {
      std::chrono::duration<double> diff;

      ros::spinOnce ();
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

    try {
      this->ros_src_sub =
          this->nh_child->subscribe (
            this->str_sub_topic_name,
            this->default_ros_queue_size,
            &NnsRosCppSubscriber::subCallbackInternal, this);
    } catch (ros::Exception &e) {
      g_critical ("%s: Failed to create a subscription for the given topic: %s",
          (this->str_nns_helper_name).c_str (), e.what ());
      throw ROS_FAILED_TO_CREATE_SUBSCRIPTION;
    }
  }

  this->looper = std::make_shared <std::thread> (&NnsRosCppSubscriber::loop,
    this);
  {
    std::lock_guard<std::mutex> lk (this->looper_m);
    this->is_looper_ready = true;
  }
  this->looper_cv.notify_all ();
}

NnsRosCppSubscriber::~NnsRosCppSubscriber ()
{
  if (!this->is_dummy)
    ros::shutdown ();
}

void
NnsRosCppSubscriber::finalize ()
{
  {
    std::unique_lock<std::mutex> lk (this->g_m);
    this->is_looper_ready = false;
    this->configured = false;
    if (!this->bag_)
      g_free (this->bag_);
  }
  this->looper->join ();
  if (!this->gasyncq)
    g_async_queue_unref (this->gasyncq);
}

void
NnsRosCppSubscriber::configure (
    const std::shared_ptr<nns_ros_bridge::Tensors> msg)
{
  guint num_tensors = static_cast<guint> (msg->tensors.size ());

  this->configs->info.num_tensors = num_tensors;
  this->configs->rate_n = this->rate * 10000;
  this->configs->rate_d = 10000;

  for (guint i = 0; i < num_tensors; ++i) {
    GstTensorInfo *info = &(this->configs->info.info[i]);
    std_msgs::MultiArrayLayout layout = msg->tensors[i].layout;

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
NnsRosCppSubscriber::subCallback (
    const std::shared_ptr<nns_ros_bridge::Tensors> msg)
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

void
NnsRosCppSubscriber::subCallbackInternal (const nns_ros_bridge::Tensors msg)
{
  auto msgSharedPtr = std::make_shared<nns_ros_bridge::Tensors> (msg);

  this->subCallback (msgSharedPtr);
}

void
NnsRosCppSubscriber::configureInternal (const nns_ros_bridge::Tensors msg)
{
  auto msgSharedPtr = std::make_shared<nns_ros_bridge::Tensors> (msg);

  this->configure (msgSharedPtr);
}

void
NnsRosCppSubscriber::loop ()
{
  if (is_dummy)
    ros::Time::init();
  ros::Rate ros_rate (this->rate);
  std::unique_lock<std::mutex> lk (this->looper_m);

  this->looper_cv.wait (lk,
      [&] () -> gboolean
      {
        return this->is_looper_ready;
      }
  );

  if (this->is_dummy) {
    rosbag::Bag *bag;

    while (true) {
      std::unique_lock<std::mutex> lk (this->g_m);
      if (!this->configured)
        std::this_thread::yield ();
      else
        break;
    }

    bag = this->getRosBagReader ();
    if (!bag)
      return;

    for(rosbag::MessageInstance const m: rosbag::View(*bag))
    {
      nns_ros_bridge::Tensors::ConstPtr msg =
          m.instantiate< nns_ros_bridge::Tensors> ();
      this->subCallbackInternal (*msg);
      ros_rate.sleep ();
      {
        std::unique_lock<std::mutex> lk (this->g_m);
        if (!this->is_looper_ready)
          break;
      }
    }
  } else {
    while (true) {
      ros::spinOnce ();
      ros_rate.sleep ();
      {
        std::unique_lock<std::mutex> lk (this->g_m);
        if (!this->is_looper_ready)
          break;
      }
    }
  }
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
    return new NnsRosCppSubscriber (node_name, topic_name, rate, timeout,
        configs, is_dummy);
  } catch (const err_code e) {
    return nullptr;
  }
}

void nns_ros_subscriber_fianlize (void *instance)
{
  NnsRosCppSubscriber *nrs_instance = (NnsRosCppSubscriber *) instance;

  nrs_instance->finalize ();

  if (nrs_instance)
    delete nrs_instance;
}

GAsyncQueue *nns_ros_subscriber_get_queue (void *instance)
{
  NnsRosCppSubscriber *nrs_instance = (NnsRosCppSubscriber *) instance;

  return nrs_instance->getQueue ();
}

void nns_ros_subscriber_put_queue (void *instance)
{
  NnsRosCppSubscriber *nrs_instance = (NnsRosCppSubscriber *) instance;

  nrs_instance->putQueue ();
}

void *
nns_ros_subscriber_open_readable_bag (void *instance, const char *path)
{
  NnsRosCppSubscriber *nrs_instance = (NnsRosCppSubscriber *) instance;
  char *path_rosbag = g_build_filename (path, NULL);
  rosbag::Bag *bag;

  if (!g_file_test (path_rosbag, G_FILE_TEST_EXISTS)) {
    g_critical ("ERROR: The given path of the rosbag file, \"%s\", is not valid.\n",
        path_rosbag);
    g_free (path_rosbag);

    return NULL;
  }

  try {
    bag = new rosbag::Bag(std::string (path_rosbag), rosbag::bagmode::Read);
  } catch (rosbag::BagException &e) {
    bag = NULL;
  }
  g_free (path_rosbag);

  nrs_instance->setRosBagReader (bag);
  for(rosbag::MessageInstance const m: rosbag::View(*bag)) {
      nns_ros_bridge::Tensors::ConstPtr msg =
          m.instantiate< nns_ros_bridge::Tensors> ();
    if (!nrs_instance->is_configured ())
      nrs_instance->configure (std::make_shared<nns_ros_bridge::Tensors> (*msg));
    else
      break;
  }

  return (void *) bag;
}
