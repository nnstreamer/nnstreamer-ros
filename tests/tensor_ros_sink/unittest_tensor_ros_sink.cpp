/**
 * @file	unittest_ros_sink.cpp
 * @date	14 February 2019
 * @brief	Unit test for tensor_ros_sink
 * @see		https://github.com/nnsuite/nnstreamer-ros
 * @author	Wook Song <wook16.song@samsung.com>
 * @bug		No known bugs.
 */

#include <gtest/gtest.h>
#include <gst/gst.h>
#include <gst/check/gstharness.h>
#include <nnstreamer/nnstreamer_plugin_api.h>

/**
 * @brief Test for setting/getting properties of tensor_ros_sink
 */
TEST (test_tensor_ros_sink, properties)
{
  const gchar defualt_name[] = "tensorrossink0";
  const gchar test_name[] = "test_tensorrossink";
  gchar *name;
  const gint64 default_max_lateness = -1;
  gint64 max_lateness, res_max_lateness;
  const gboolean default_qos = TRUE;
  gboolean qos, res_qos;
  const guint default_signal_rate = 0;
  guint signal_rate, res_signal_rate;
  const gboolean default_emit_signal = TRUE;
  gboolean emit_signal, res_emit_signal;
  const gboolean default_silent = TRUE;
  gboolean silent, res_silent;
  GstHarness *hrnss;

  hrnss = gst_harness_new ("tensor_ros_sink");
  ASSERT_TRUE (hrnss != NULL);

  /** default name is "tensorrossink0" (default_name) */
  name = gst_element_get_name (hrnss->element);
  ASSERT_TRUE (name != NULL);
  EXPECT_STREQ (defualt_name, name);
  g_free (name);

  gst_element_set_name (hrnss->element, test_name);
  name = gst_element_get_name (hrnss->element);
  ASSERT_TRUE (name != NULL);
  EXPECT_STREQ (test_name, name);
  g_free (name);

  /** default max-lateness is -1 (default_max_lateness), which means unlimited time */
  g_object_get (hrnss->element, "max-lateness", &max_lateness, NULL);
  EXPECT_EQ (default_max_lateness, max_lateness);

  max_lateness = 30 * GST_MSECOND;
  g_object_set (hrnss->element, "max-lateness", max_lateness, NULL);
  g_object_get (hrnss->element, "max-lateness", &res_max_lateness, NULL);
  EXPECT_EQ (max_lateness, res_max_lateness);

  /** default qos is TRUE (default_qos) */
  g_object_get (hrnss->element, "qos", &qos, NULL);
  EXPECT_EQ (default_qos, qos);

  g_object_set (hrnss->element, "qos", !default_qos, NULL);
  g_object_get (hrnss->element, "qos", &res_qos, NULL);
  EXPECT_EQ (!default_qos, res_qos);

  /** default signal-rate is 0 (default_signal_rate) */
  g_object_get (hrnss->element, "signal-rate", &signal_rate, NULL);
  EXPECT_EQ (default_signal_rate, signal_rate);

  signal_rate += 10;
  g_object_set (hrnss->element, "signal-rate", signal_rate, NULL);
  g_object_get (hrnss->element, "signal-rate", &res_signal_rate, NULL);
  EXPECT_EQ (signal_rate, res_signal_rate);

  /** default emit-signal is TRUE */
  g_object_get (hrnss->element, "emit-signal", &emit_signal, NULL);
  EXPECT_EQ (default_emit_signal, emit_signal);

  g_object_set (hrnss->element, "emit-signal", !default_emit_signal, NULL);
  g_object_get (hrnss->element, "emit-signal", &res_emit_signal, NULL);
  EXPECT_EQ (!default_emit_signal, res_emit_signal);

   /** default silent is TRUE */
  g_object_get (hrnss->element, "silent", &silent, NULL);
  EXPECT_EQ (default_silent, silent);

  g_object_set (hrnss->element, "silent", !default_silent, NULL);
  g_object_get (hrnss->element, "silent", &res_silent, NULL);
  EXPECT_EQ (!default_silent, res_silent);
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
