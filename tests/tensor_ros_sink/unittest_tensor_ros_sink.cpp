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

  gst_harness_teardown (hrnss);
}

int cnt_new_data_signal = 0;
/**
 * @brief Callback for the "new-data" signal emitted by tensor_ros_sink
 */
static void callback_sink_new_data_signal (GstElement* object,
    gpointer user_data)
{
  cnt_new_data_signal++;
}

#define SIZE_STR_BUF 255
#define TEST_SINK_SIG_NEW_DATA(nns_data_t, data_t) \
    TEST (test_tensor_ros_sink, signal_new_data_##data_t) { \
      const guint num_buffers = 10; \
      const guint dims[NNS_TENSOR_RANK_LIMIT] = {3, 480, 640, 10}; \
      gchar str_dims[SIZE_STR_BUF]; \
      GstHarness *hrnss; \
      GstTensorConfig config; \
      GstBuffer *in_buf; \
      GstMemory *mem; \
      GstMapInfo info; \
      gsize data_size_in_bytes, num_data_in_buf; \
      gulong sig_id; \
      guint i, b; \
      \
      hrnss = gst_harness_new ("tensor_ros_sink"); \
      ASSERT_TRUE (hrnss != NULL); \
      \
      cnt_new_data_signal = 0; \
      sig_id = g_signal_connect (hrnss->element, "new-data", \
          G_CALLBACK (callback_sink_new_data_signal), NULL); \
          ASSERT_TRUE (sig_id > 0); \
      \
      snprintf (str_dims, SIZE_STR_BUF, "%u:%u:%u:%u", \
          dims[0], dims[1], dims[2], dims[3]); \
      config.info.type = nns_data_t; \
      gst_tensor_parse_dimension (str_dims, config.info.dimension); \
      config.rate_n = 0; \
      config.rate_d = 1; \
      \
      gst_harness_set_src_caps (hrnss, gst_tensor_caps_from_config (&config)); \
      data_size_in_bytes = gst_tensor_info_get_size (&config.info); \
      num_data_in_buf = data_size_in_bytes / sizeof(data_t); \
      \
      for (b = 0; b < num_buffers; b++) { \
        \
        in_buf = gst_harness_create_buffer (hrnss, data_size_in_bytes); \
        \
        mem = gst_buffer_peek_memory (in_buf, 0); \
        ASSERT_TRUE (gst_memory_map (mem, &info, GST_MAP_WRITE)); \
        \
        for (i = 0; i <  num_data_in_buf; i++) { \
          data_t value = (i + 1) * (b + 1); \
          ((data_t *) info.data)[i] = value; \
        } \
        \
        gst_memory_unmap (mem, &info); \
        EXPECT_EQ (gst_harness_push (hrnss, in_buf), GST_FLOW_OK); \
      } \
      EXPECT_EQ (num_buffers, cnt_new_data_signal); \
      \
      g_signal_handler_disconnect (hrnss->element, sig_id); \
      gst_harness_teardown (hrnss); \
    \
    }

/**
 * @brief Tests for the "new-data" signal emitted by tensor_ros_sink
 */
TEST_SINK_SIG_NEW_DATA (_NNS_UINT8, uint8_t)
TEST_SINK_SIG_NEW_DATA (_NNS_UINT16, uint16_t)
TEST_SINK_SIG_NEW_DATA (_NNS_UINT32, uint32_t)
TEST_SINK_SIG_NEW_DATA (_NNS_UINT64, uint64_t)
TEST_SINK_SIG_NEW_DATA (_NNS_INT8, int8_t)
TEST_SINK_SIG_NEW_DATA (_NNS_INT16, int16_t)
TEST_SINK_SIG_NEW_DATA (_NNS_INT32, int32_t)
TEST_SINK_SIG_NEW_DATA (_NNS_INT64, int64_t)
TEST_SINK_SIG_NEW_DATA (_NNS_FLOAT32, float)
TEST_SINK_SIG_NEW_DATA (_NNS_FLOAT64, double)

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
