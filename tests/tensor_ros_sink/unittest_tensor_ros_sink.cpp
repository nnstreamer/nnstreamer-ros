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
