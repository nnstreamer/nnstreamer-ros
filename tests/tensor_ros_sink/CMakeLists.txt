SET(PKG_MODULES
  gstreamer-base-1.0
  gstreamer-check-1.0
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

PKG_CHECK_MODULES(PKGS_TEST REQUIRED ${PKG_MODULES})
ADD_EXECUTABLE(unittest_tensor_ros_sink unittest_tensor_ros_sink.cpp ${GTEST_SRC})
TARGET_INCLUDE_DIRECTORIES(unittest_tensor_ros_sink PRIVATE
  ${PKGS_COMMON_INCLUDE_DIRS}
  ${PKGS_TEST_INCLUDE_DIRS}
  ${GTEST_INC}
)

TARGET_LINK_LIBRARIES(unittest_tensor_ros_sink
  ${PKGS_COMMON_LIBRARIES}
  ${PKGS_TEST_LIBRARIES}
  ${GTEST_LINK_LIBS}
)

SET(TEST_SETS_SINK
  properties
  signal_new_data_uint8_t
  signal_new_data_uint16_t
  signal_new_data_uint32_t
  signal_new_data_uint64_t
  signal_new_data_int8_t
  signal_new_data_int16_t
  signal_new_data_int32_t
  signal_new_data_int64_t
  signal_new_data_float
  signal_new_data_double
)

FOREACH(eachtest IN LISTS TEST_SETS_SINK)
  ADD_TEST(NAME unittest_tensor_ros_sink.${eachtest}
    COMMAND unittest_tensor_ros_sink --gtest_filter=test_tensor_ros_sink.${eachtest}
  )
ENDFOREACH()
