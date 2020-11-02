/**
 * Copyright (C) 2019 Samsung Electronics Co., Ltd. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; 
 * version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 */
/**
 * @file   nns_ros_subscriber.h
 * @author Sangjung Woo <sangjung.woo@samsung.com>
 * @date   03/06/2019
 * @brief  Abstract class for subscribing target Ros topic
 *
 * @bug     No known bugs.
 */
#ifndef __NNS_ROS_SUBSCRIBER_H__
#define __NNS_ROS_SUBSCRIBER_H__

#include <glib-2.0/glib.h>
#include <nnstreamer/tensor_typedef.h>
#include <nnstreamer/nnstreamer_plugin_api.h>

/* @todo Use common err_code */
typedef enum _err_code {
  ROS_INVALID_OPTION,
  ROS_FAILED_TO_CREATE_SUBSCRIPTION,
  ROS_SUBSCRIPTION_TIMED_OUT,
  ROS1_UNDEFINED_ROS_MASTER_URI,
  ROS1_FAILED_TO_CONNECT_ROSCORE,
  ROS2_FAILED_TO_CREATE_NODE,
} err_code;

#ifdef __cplusplus
#include <chrono>
#include <condition_variable>
#include <string>
#include <thread>

template <class T> class NnsRosSubscriber
{
public:
  NnsRosSubscriber (const gchar *node_name, const char *topic_name,
      const gdouble rate, const gdouble timeout, GstTensorsConfig *configs,
      gboolean is_dummy)
  {
    this->is_configured = false;
    this->num_tensors = 0;
    this->configs = configs;
    this->str_node_name = std::string (node_name);
    this->str_sub_topic_name = std::string (topic_name);
    this->is_looper_ready = false;
    this->rate = rate;
    this->timeout = std::chrono::duration<double> (timeout);
    this->is_dummy = is_dummy;
    this->gasyncq = g_async_queue_new ();
  }
  virtual ~NnsRosSubscriber () { };

  const gchar *getSubTopicName ()
  {
    return this->str_sub_topic_name.c_str ();
  }

  static const gchar *getHelperName ()
  {
    return NnsRosSubscriber::str_nns_helper_name.c_str ();
  }

  GAsyncQueue *getQueue ()
  {
    g_async_queue_ref (this->gasyncq);
    return this->gasyncq;
  }

  void putQueue ()
  {
    g_async_queue_unref (this->gasyncq);
  }

  virtual void subCallback (const std::shared_ptr<T> msg) = 0;
  virtual void configure (const std::shared_ptr<T> msg) = 0;
  virtual void finalize () = 0;

protected:
  NnsRosSubscriber () {};

  std::mutex g_m;
  // Variables for GST configuration
  bool is_configured;
  guint num_tensors;
  GstTensorsConfig *configs;

  // Variables for ROS configuration
  static std::string str_nns_helper_name;
  std::string str_node_name;
  std::string str_sub_topic_name;
  gdouble rate;
  gboolean is_dummy;

  // Variables for subscribing ROS topic
  static const size_t default_ros_queue_size = 15;
  std::shared_ptr<std::thread> looper;
  std::mutex looper_m;
  bool is_looper_ready;
  std::condition_variable looper_cv;
  virtual void loop () = 0;
  std::chrono::duration<double> timeout;
  GAsyncQueue *gasyncq;
};

template <class T>
std::string NnsRosSubscriber<T>::str_nns_helper_name = "nns_ros_subscriber";

extern "C"
{
#endif /* __cplusplus */

void *nns_ros_subscriber_init (const gchar *node_name, const char *topic_name,
      const gdouble rate, const gdouble timeout, GstTensorsConfig *configs,
      gboolean is_dummy);
void nns_ros_subscriber_fianlize (void *instance);
GAsyncQueue *nns_ros_subscriber_get_queue (void *instance);
void nns_ros_subscriber_put_queue (void *instance);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __NNS_ROS_SUBSCRIBER_H__ */
