/**
 * @file	unittest_ros_src.cpp
 * @date	18 March 2019
 * @brief	Unit test for tensor_ros_src
 * @see		https://github.com/nnsuite/nnstreamer-ros
 * @author	Sangjung Woo <sangjung.woo@samsung.com>
 * @bug		No known bugs.
 */
#include <gtest/gtest.h>
#include <gst/gst.h>
#include <gst/check/gstharness.h>

#if (!(defined(WITH_ROS1) || defined(WITH_ROS2)))
#error "One of WITH_ROS or WITH_ROS2 should be defined."
#endif /** (!(defined(WITH_ROS) || defined(WITH_ROS2)) */

#if defined(WITH_ROS2)
#define LAUNCH_LINE_SRC "tensor_ros2_src"
#define ELMNT_NAME_SRC "tensor_ros2_src"
#elif defined(WITH_ROS1)
#define LAUNCH_LINE_SRC "tensor_ros_src"
#define ELMNT_NAME_SRC "tensor_ros_src"
#endif /** WITH_ROS1 */

TEST (test_tensor_ros_src, properties)
{
  const gdouble default_freqrate = 1.0;
  GstHarness *hrnss = NULL;
  GstElement *rossrc = NULL;
  gchar *name;
  const gchar default_name[] = "tensorrossrc0";
  const gboolean default_silent = TRUE;
  gboolean silent, ret_silent;
  const gchar *topic_name = "test_topic";
  gchar *ret_topic_name;
  gdouble freqrate;
  gdouble ret_freqrate;
  const gdouble defualt_timeout = 5.0;
  gdouble timeout;
  gdouble ret_timeout;
  const gboolean default_load_rosbag = FALSE;
  gboolean ret_load_rosbag;
  gchar *ret_location;

  /* setup */
  hrnss = gst_harness_new_empty ();
  ASSERT_TRUE (hrnss != NULL);
  gst_harness_add_parse (hrnss, LAUNCH_LINE_SRC);
  rossrc = gst_harness_find_element (hrnss, ELMNT_NAME_SRC);
  ASSERT_TRUE (rossrc != NULL);

  /* check the default name */
  name = gst_element_get_name (rossrc);
  ASSERT_TRUE (name != NULL);
  EXPECT_STREQ (default_name, name);
  g_free (name);

  /* silent mode test */
  g_object_get (rossrc, "silent", &ret_silent, NULL);
  EXPECT_EQ (ret_silent, default_silent);
  silent = FALSE;
  g_object_set (rossrc, "silent", silent, NULL);
  g_object_get (rossrc, "silent", &ret_silent, NULL);
  EXPECT_EQ (silent, ret_silent);

  /* topic name test */
  g_object_set (rossrc, "topic", topic_name, NULL);
  g_object_get (rossrc, "topic", &ret_topic_name, NULL);
  EXPECT_STREQ (topic_name, ret_topic_name);
  g_free (ret_topic_name);

  /* freqrate test */
  g_object_get (rossrc, "rate", &ret_freqrate, NULL);
  EXPECT_DOUBLE_EQ (ret_freqrate, default_freqrate);

  freqrate = 10.0;
  g_object_set (rossrc, "rate", freqrate, NULL);
  g_object_get (rossrc, "rate", &ret_freqrate, NULL);
  EXPECT_DOUBLE_EQ (ret_freqrate, freqrate);

  g_object_get (rossrc, "timeout", &ret_timeout, NULL);
  EXPECT_DOUBLE_EQ (ret_timeout, defualt_timeout);
  timeout = 1.0;
  g_object_set (rossrc, "timeout", timeout, NULL);
  g_object_get (rossrc, "timeout", &ret_timeout, NULL);
  EXPECT_DOUBLE_EQ (ret_timeout, timeout);

  g_object_get (rossrc, "enable-load-rosbag", &ret_load_rosbag, NULL);
  EXPECT_EQ (ret_load_rosbag, default_load_rosbag);
  g_object_set (rossrc, "enable-load-rosbag", !default_load_rosbag, NULL);
  g_object_get (rossrc, "enable-load-rosbag", &ret_load_rosbag, NULL);
  EXPECT_EQ (ret_load_rosbag, !default_load_rosbag);

  g_object_get (rossrc, "location", &ret_location, NULL);
  EXPECT_TRUE (ret_location == NULL);
  g_object_set (rossrc, "location", "A-ROSBAG-PATH", NULL);
  g_object_get (rossrc, "location", &ret_location, NULL);
  EXPECT_STREQ (ret_location, "A-ROSBAG-PATH");

  /* teardown */
  gst_harness_teardown (hrnss);
}

/**
 * @brief Main function for unit test.
 */
int
main (int argc, char **argv)
{
  testing::InitGoogleTest (&argc, argv);

  gst_init (&argc, &argv);

  return RUN_ALL_TESTS ();
}
