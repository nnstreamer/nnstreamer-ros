/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * @file        nns_roscpp_publisher.cc
 * @date        11/19/2018
 * @brief       A helper class for publishing ROS1 (i.e., roscpp) topic
 *              via NNStreamer plugins
 * @see         https://github.com/nnstreamer/nnstreamer-ros
 * @author      Wook Song <wook16.song@samsung.com>
 * @bug         No known bugs except for NYI items
 *
 * This class bridges between the NNStreamer (C) and ROS1 frameworks
 * (ROSCPP/C++) by implementing the NnsRosPublisher class.
 *
 */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <time.h>

#include "nns_ros_publisher.h"
#include "nns_ros_bridge/Tensors.h"

const char BASE_NODE_NAME[] = "tensor_ros_sink";

class NnsRosCppPublisher
    : public NnsRosPublisher<std_msgs::MultiArrayLayout, guint>
{
public:
  NnsRosCppPublisher (const char *node_name, const char *topic_name,
      gboolean is_dummy);
  ~NnsRosCppPublisher ();

  gboolean publish (const guint num_tensors, const GstTensorMemory *tensors_mem,
      void *bag) final;

private:
  // Variables for ROS configuration
  ros::NodeHandle *nh_parent;
  ros::NodeHandle *nh_child;
  ros::Publisher ros_sink_pub;
};

template <>
std::string
    NnsRosPublisher<std_msgs::MultiArrayLayout, guint>::str_nns_helper_name =
        "nns_roscpp_publisher";

/**
 * @brief	The only public constructor of this class
 * @param[in] node_name The name of the ROS node under /tensor_ros_sink
 * @param[in] topic_name The name of topic what this class concerns
 * @param[in] is_dummy If TRUE, create the instance without roscore connection
 * @return None
 */
NnsRosCppPublisher::NnsRosCppPublisher (const char *node_name,
    const char *topic_name, gboolean is_dummy)
    : NnsRosPublisher (node_name, topic_name, is_dummy)
{
  /* ROS initialization requires command-line arguments */
  int dummy_argc = 0;
  char **dummy_argv = NULL;

  if (!is_dummy) {
    if (getenv ("ROS_MASTER_URI") == NULL) throw ROS1_UNDEFINED_ROS_MASTER_URI;

    ros::init (dummy_argc, dummy_argv, this->str_node_name);

    if (!ros::master::check ()) throw ROS1_FAILED_TO_CONNECT_ROSCORE;

    this->nh_parent = new ros::NodeHandle (BASE_NODE_NAME);
    this->nh_child =
        new ros::NodeHandle (*(this->nh_parent), this->str_node_name);

    /**
     * The name of published topic would be
     * /tensor_ros_sink/PID_${PID}/${ElementNameOfTensorRosSink}
     */
    this->ros_sink_pub = this->nh_child->advertise<nns_ros_bridge::Tensors> (
        this->str_pub_topic_name, default_q_size);
  }
}

NnsRosCppPublisher::~NnsRosCppPublisher ()
{
  if (!this->is_dummy) {
    delete this->nh_child;
    delete this->nh_parent;
    ros::shutdown ();
  }
}

/**
 * @brief	A method for publishing a ROS topic that contains the tensors
 * passed by the NNStreamer framework
 * @param[in] num_tensors The number of tensors included in tensors_mem (for the
 * verification purpose)
 * @param[in] tensors_mem The pointer of containers consists of information and
 * raw data of tensors
 * @return TRUE if the configuration is successfully done
 */
gboolean NnsRosCppPublisher::publish (const guint num_tensors,
    const GstTensorMemory *tensors_mem, void *bag)
{
  nns_ros_bridge::Tensors tensors_msg;

  g_return_val_if_fail (this->ready_to_pub, FALSE);
  g_return_val_if_fail (num_tensors == this->num_of_tensors_pub, FALSE);

  for (guint i = 0; i < this->num_of_tensors_pub; ++i) {
    std_msgs::UInt8MultiArray each_tensor;
    uint8_t *src_data = (uint8_t *) tensors_mem[i].data;

    each_tensor.layout = this->topic_layouts_pub[i];
    each_tensor.data.assign (src_data, src_data + tensors_mem[i].size);

    tensors_msg.tensors.push_back (each_tensor);
  }

  if (!this->is_dummy) {
    g_return_val_if_fail (ros::ok (), FALSE);
    this->ros_sink_pub.publish (tensors_msg);
    ros::spinOnce ();
  }

  if (bag != NULL) {
    rosbag::Bag *bag_instance = (rosbag::Bag *) bag;

    // TODO: It is better to use the gst-timestamp value corresponding to each
    // tensor_msg
    timespec ts_now;
    clock_gettime (CLOCK_REALTIME, &ts_now);
    ros::Time now_time (ts_now.tv_sec, ts_now.tv_nsec);
    bag_instance->write (this->str_pub_topic_name, now_time, tensors_msg);
  }

  return TRUE;
}

/**
 * C functions for NNStreamer plugins that want to publish or subscribe
 * ROS topics. nns_ros_publisher_init() should be invoked before configuring the
 * tensors information to publish or subscribe. Each NNStreamer plugin which
 * uses this class should be holding the instance of NnsRosCppPublisher (i.e.,
 * the void type pointer what the nns_ros_publisher_init() function returns).
 */
void *nns_ros_publisher_init (const char *node_name, const char *topic_name,
    gboolean is_dummy)
{
  try {
    return new NnsRosCppPublisher (node_name, topic_name, is_dummy);
  } catch (const err_code e) {
    switch (e) {
      case ROS1_UNDEFINED_ROS_MASTER_URI:
        g_critical ("%s: ROS_MASTER_URI is not defined in the environment\n",
            NnsRosCppPublisher::getHelperName ());
        break;

      case ROS1_FAILED_TO_CONNECT_ROSCORE:
        g_critical (
            "%s: failed to connect to master: please make sure roscore is "
            "running\n", NnsRosCppPublisher::getHelperName ());
        break;

      default:
        break;
    }
    return NULL;
  }
}

void nns_ros_publisher_finalize (void *instance)
{
  NnsRosCppPublisher *nrp_instance = (NnsRosCppPublisher *) instance;
  if (nrp_instance != NULL) delete nrp_instance;
}

gboolean nns_ros_publisher_publish (void *instance, const guint num_tensors,
    const GstTensorMemory *tensors_mem, void *bag)
{
  NnsRosCppPublisher *nrp_instance = (NnsRosCppPublisher *) instance;
  rosbag::Bag *bag_instance = (rosbag::Bag *) bag;

  return nrp_instance->publish (num_tensors, tensors_mem, bag_instance);
}

gboolean nns_ros_publisher_set_pub_topic (void *instance,
    const GstTensorsConfig *conf)
{
  NnsRosCppPublisher *nrp_instance = (NnsRosCppPublisher *) instance;

  return nrp_instance->setPubTopicInfo<std_msgs::MultiArrayDimension> (conf);
}

const gchar *nns_ros_publisher_get_pub_topic_name (void *instance)
{
  NnsRosCppPublisher *nrp_instance = (NnsRosCppPublisher *) instance;

  return nrp_instance->getPubTopicName ();
}

void *nns_ros_publisher_open_writable_bag (void *instance, const char *name)
{
  rosbag::Bag *bag;
  char *path_rosbag;

  if (name == NULL || name[0] == '\0') {
    path_rosbag = g_strdup_printf (
        "%s.bag", nns_ros_publisher_get_pub_topic_name (instance));
  } else {
    path_rosbag = g_strdup (name);
  }

  try {
    bag = new rosbag::Bag(std::string (path_rosbag), rosbag::bagmode::Write);
  } catch (rosbag::BagException &e) {
    bag = NULL;
  }
  g_free (path_rosbag);

  return (void *) bag;
}

void nns_ros_publisher_close_bag(void *bag)
{
  rosbag::Bag *rosbag = (rosbag::Bag *) bag;

  if (rosbag != NULL) {
    rosbag->close ();
  }
}
