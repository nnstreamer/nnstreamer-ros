/* SPDX-License-Identifier: LGPL-2.1-only */
/**
 * NNStreamer-ROS: NNStreamer extension packages for ROS/ROS2 support
 * Copyright (C) 2020 Wook Song <wook16.song@samsung.com>
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * @file        nns_ros_publisher.h
 * @date        12 Oct 2020
 * @brief       A virtual class for publishing the ROS/ROS2 topic
 *              via NNStreamer plugins
 * @see         https://github.com/nnstreamer/nnstreamer-ros
 * @author      Wook Song <wook16.song@samsung.com>
 * @bug         No known bugs except for NYI items
 */

#ifndef _NNS_ROS_PUBLISHER_H_
#define _NNS_ROS_PUBLISHER_H_

#include <glib-2.0/glib.h>
#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>

typedef enum _err_code {
  ROS1_UNDEFINED_ROS_MASTER_URI,
  ROS1_FAILED_TO_CONNECT_ROSCORE,
  ROS2_FAILED_TO_CREATE_NODE,
  ROS2_FAILED_TO_CREATE_PUBLISHER,
} err_code;

#ifdef __cplusplus
#include <string>

template <class T, typename S> class NnsRosPublisher
{
public:
  NnsRosPublisher (const char *node_name, const char *topic_name,
      gboolean is_dummy, S q_size = 15)
  {
    this->str_node_name = std::string (node_name);
    this->str_pub_topic_name = std::string (topic_name);
    this->ready_to_pub = FALSE;
    this->is_dummy = is_dummy;
    this->default_q_size = q_size;
  }
  virtual ~NnsRosPublisher () { };

  const gchar *getPubTopicName ()
  {
    return this->str_pub_topic_name.c_str ();
  }

  static const gchar *getHelperName ()
  {
    return NnsRosPublisher::str_nns_helper_name.c_str ();
  }

  virtual gboolean publish (const guint num_tensors,
      const GstTensorMemory *tensors_mem, void *bag) = 0;

  /**
   * @brief	A method for configuraing topic information to publish
   * @param[in] conf The configuration information at the NNStreamer side
   * @return TRUE if the configuration is successfully done
  */
  template <class U>
  gboolean setPubTopicInfo (const GstTensorsConfig *conf)
  {
    const GstTensorsInfo *tensors_info = &conf->info;

    g_return_val_if_fail (tensors_info->num_tensors <= NNS_TENSOR_SIZE_LIMIT,
        FALSE);

    this->num_of_tensors_pub = tensors_info->num_tensors;

    for (guint i = 0; i < this->num_of_tensors_pub; ++i) {
      gsize tensor_size = gst_tensor_info_get_size (&tensors_info->info[i]);
      guint each_dim_stride = tensor_size;

      for (gint j = (NNS_TENSOR_RANK_LIMIT - 1); j >= 0; --j) {
        U each_dim;
        /**
        * FIXME: Since the type of 'stride' of std_msgs::MultiArrayDimension is
        * unsigned int,conversion from size_t to unsigned int is requried.
        */
        each_dim.label = gst_tensor_get_type_string(tensors_info->info[i].type);
        each_dim.size =
            tensors_info->info[i].dimension[j];
        if (j != (NNS_TENSOR_RANK_LIMIT - 1)) {
          each_dim_stride /= tensors_info->info[i].dimension[j + 1];
        }
        each_dim.stride = each_dim_stride;

        this->topic_layouts_pub[i].dim.push_back (each_dim);
      }

      g_return_val_if_fail (
          this->topic_layouts_pub[i].dim.at(0).stride == tensor_size, FALSE
          );
    }

    this->ready_to_pub = TRUE;

    return ready_to_pub;
  }

protected:
  // Variables for ROS configuration
  static std::string str_nns_helper_name;
  std::string str_node_name;
  std::string str_pub_topic_name;
  gboolean is_dummy;
  S default_q_size;
  // Variables for publish ROS topic
  gboolean ready_to_pub;
  guint num_of_tensors_pub;
  T topic_layouts_pub[NNS_TENSOR_SIZE_LIMIT];
};

template <class T, typename S>
std::string NnsRosPublisher<T, S>::str_nns_helper_name = "nns_ros_publisher";

extern "C"
{
#endif /* __cplusplus */

void *nns_ros_publisher_init (const char *node_name, const char *topic_name,
    gboolean is_dummy);
void nns_ros_publisher_finalize (void *instance);
gboolean nns_ros_publisher_publish (void *instance, const guint num_tensors,
    const GstTensorMemory *tensors_mem, void *bag);
gboolean nns_ros_publisher_set_pub_topic (void *instance,
    const GstTensorsConfig *conf);
const gchar *nns_ros_publisher_get_pub_topic_name (void *instance);
void *nns_ros_publisher_open_writable_bag (void *instance, const char *name);
void nns_ros_publisher_close_bag (void *instance, void *bag);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _NNS_ROS_PUBLISHER_H_ */
