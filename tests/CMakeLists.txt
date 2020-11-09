SET(PKG_MODULES_TESTS
  gstreamer-check-1.0
  gstreamer-plugins-base-1.0
)

FIND_LIBRARY(GTEST_LIB gtest)
IF(GTEST_LIB OR TIZEN)
  # Use GTEST shared lib (Tizen supplied gtest as shared lib)
  SET(GTEST_LINK_LIBS gtest pthread)
  SET(GTEST_SRC "")
  SET(GTEST_INC "")
ELSE(GTEST_LIB OR TIZEN)
  IF(NOT EXISTS /usr/src/gtest/src/gtest-all.cc)
    MESSAGE(FATAL_ERROR "You need to install libgtest-dev or (supplied with GTEST_LIB) libgtest.so.")
  ENDIF (NOT EXISTS /usr/src/gtest/src/gtest-all.cc)
  SET(GTEST_LINK_LIBS pthread)
  SET(GTEST_SRC /usr/src/gtest/src/gtest-all.cc)
  SET(GTEST_INC /usr/src/gtest)
ENDIF(GTEST_LIB OR TIZEN)

PKG_CHECK_MODULES(PKGS_TEST REQUIRED ${PKG_MODULES_TESTS})

SET(ELMNT_NAME_SINK tensor_ros2_sink)
SET(ELMNT_NAME_SRC tensor_ros2_src)
IF(WITH_ROS1)
  SET(ELMNT_NAME_SINK tensor_ros_sink)
  SET(ELMNT_NAME_SRC tensor_ros_src)
ENDIF(WITH_ROS1)

ADD_SUBDIRECTORY(tensor_ros_sink)
ADD_SUBDIRECTORY(tensor_ros_src)
