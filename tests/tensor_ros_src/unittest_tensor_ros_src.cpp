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

#define LAUNCHLINE_ROSSRC "tensor_ros_src"
#define TARGET_ELEMENT_NAME "tensor_ros_src"

const guint64 DEFAULT_FREQRATE = 1;
const gchar *DATATYPES[] = { "int32", "uint32",
                            "int8", "uint8",
                            "int16", "uint16",
                            "int64", "uint64",
                            "float32" , "float64"};

TEST (test_tensor_ros_src, properties)
{
  GstHarness *hrnss = NULL;
  GstElement *rossrc = NULL;
  gchar *name;
  const gchar default_name[] = "tensorrossrc0";
  const gboolean default_silent = TRUE;
  gboolean silent, ret_silent;
  const gchar *topic_name = "test_topic";
  gchar *ret_topic_name;
  guint64 freqrate;
  guint64 ret_freqrate;
  gchar *ret_datatype;

  /* setup */
  hrnss = gst_harness_new_empty ();
  ASSERT_TRUE (hrnss != NULL);
  gst_harness_add_parse (hrnss, LAUNCHLINE_ROSSRC);
  rossrc = gst_harness_find_element (hrnss, TARGET_ELEMENT_NAME);
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
  g_object_get (rossrc, "freqrate", &ret_freqrate, NULL);
  EXPECT_EQ (ret_freqrate, DEFAULT_FREQRATE);

  freqrate = 10;
  g_object_set (rossrc, "freqrate", freqrate, NULL);
  g_object_get (rossrc, "freqrate", &ret_freqrate, NULL);
  EXPECT_EQ (ret_freqrate, freqrate);

  /* datatype test */
  g_object_get (rossrc, "datatype", &ret_datatype, NULL);
  EXPECT_STREQ (ret_datatype, DATATYPES[0]);

  unsigned int size = sizeof(DATATYPES) / sizeof(DATATYPES[0]);
  for (unsigned int i = 0; i < size; ++i) {
    g_object_set (rossrc, "datatype", DATATYPES[i], NULL);
    g_object_get (rossrc, "datatype", &ret_datatype, NULL);
    EXPECT_STREQ (ret_datatype, DATATYPES[i]);
  }

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
