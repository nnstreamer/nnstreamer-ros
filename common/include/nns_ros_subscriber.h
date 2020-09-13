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

#include <glib.h>
#include <ros/ros.h>

class NnsRosSubscriber {
public:
  NnsRosSubscriber (const gchar *node_name,
    const gchar *topic_name,
    const gulong rate_usec = G_USEC_PER_SEC,
    int argc = 0,
    gchar **argv = NULL);
  ~NnsRosSubscriber ();

  virtual int RegisterCallback (ros::NodeHandle *nh, ros::Subscriber *sub) = 0;
  int Start (GThread ** gthread_obj);
  int RequestStop ();

protected:
  gchar *node_name;
  gchar *topic_name;

private:
  NnsRosSubscriber () {};

  static gpointer ThreadFunc (gpointer userdata);

  ros::NodeHandle *nh;
  ros::Subscriber sub;
  gulong rate_usec;
  gboolean request_stop;
};
#endif /* __NNS_ROS_SUBSCRIBER_H__ */
