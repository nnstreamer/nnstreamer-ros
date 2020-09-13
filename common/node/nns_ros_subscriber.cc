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
 * @file   nns_ros_subscriber.cc
 * @author Sangjung Woo <sangjung.woo@samsung.com>
 * @date   03/06/2019
 * @brief  Abstract class for subscribing target Ros topic
 *
 * @bug     No known bugs.
 */
#include "nns_ros_subscriber.h"

/**
 * @brief	The public constructor of this class
 * @param[in] node_name : The name of the Ros client node
 * @param[in] topic_name : The name of target topic to subscribe
 * @param[in] rate_usec : The sampling rate to check the published Ros topic (Default: 1 sec)
 * @param[in] argc : argc parameter which is used for ros::init (Default: 0)
 * @param[in] argv : argv parameter which is used for ros::init (Default: NULL)
 * @return None
 */
NnsRosSubscriber::NnsRosSubscriber (const gchar *node_name,
    const gchar *topic_name,
    const gulong rate_usec,
    int argc,
    gchar **argv)
{
  this->node_name = g_strdup (node_name);
  this->topic_name = g_strdup (topic_name);
  this->rate_usec = rate_usec;
  this->request_stop = false;

  ros::init (argc, argv, this->node_name);
}

/**
 *  @brief The destructor of this class
 */
NnsRosSubscriber::~NnsRosSubscriber ()
{
  if (this->node_name)
    g_free (this->node_name);
  
  if (this->topic_name)
    g_free (this->topic_name);
}

/**
 * @brief	A method for starting thread to subscribe target Ros topic
 * @param[in] gthread_obj : GThread pointer
 * @return 0 when thread is created and running
 */
int
NnsRosSubscriber::Start (GThread ** gthread_obj)
{
  this->nh = new ros::NodeHandle();

  /* Call the subclass's method */
  RegisterCallback (this->nh, &this->sub);
  *gthread_obj = g_thread_new ("RosSubThread", (GThreadFunc)NnsRosSubscriber::ThreadFunc, this);

  return 0;
}

/**
 * @brief	A metohd to stop running thread for subscribing Ros topic
 * @return 0 if request_stop is set as true
 */
int
NnsRosSubscriber::RequestStop ()
{
  this->request_stop = true;
  return 0;
}

/**
 * @brief	A method for checking published Ros topic on a desinated cycle
 * @param[in] gpointer which points NnsRosSubscriber instance
 * @return NULL when running thread exits
 */
gpointer
NnsRosSubscriber::ThreadFunc (gpointer userdata)
{
  NnsRosSubscriber *rossub = static_cast <NnsRosSubscriber *> (userdata);
  while (TRUE) {
    if (rossub->request_stop) {
      g_printerr ("Sub thread is going to stop!\n");
      break;
    }

    /* check the ros topic */
    ros::spinOnce();
    g_usleep (rossub->rate_usec);
  }
  g_thread_exit (GINT_TO_POINTER (0));

  return NULL;
}
