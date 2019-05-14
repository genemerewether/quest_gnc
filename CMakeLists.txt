cmake_minimum_required(VERSION 2.8.3)
project(quest_gnc)

if(DEFINED NO_ROS)
  set(USE_ROS FALSE)
else()
  set(USE_ROS TRUE)
endif()
set(__ignoreMe ${NO_ROS})

if (USE_ROS)
  find_package(catkin_simple REQUIRED)
  catkin_simple()
endif()

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/../tools/pybind11/CMakeLists.txt")
  set(HAVE_PYBIND11 TRUE)
  message(STATUS "Found pybind11; generating Python bindings")
endif()

############
# PROTOBUF #
############
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
#set(PROTO_DEFNS proto/quest_gnc/*.proto)
set(ADDITIONAL_LIBRARIES "")

find_package(protobuf_catkin QUIET)
if (protobuf_catkin_FOUND)
    message(STATUS "Using protobuf_catkin")
    list(APPEND catkin_INCLUDE_DIRS ${protobuf_catkin_INCLUDE_DIRS})
    list(APPEND catkin_LIBRARIES ${protobuf_catkin_LIBRARIES})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    #PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
    set(ADDITIONAL_LIBRARIES ${protobuf_catkin_LIBRARIES})
else()
  message(STATUS "Using system protobuf")
  find_package(Protobuf REQUIRED)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  #PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(ADDITIONAL_LIBRARIES ${PROTOBUF_LIBRARY})
endif()

####################
# SET SOURCE FILES #
####################

set("${PROJECT_NAME}_SRCS"
  src/diffeo/body_frame.cpp
  src/diffeo/rates.cpp
  src/ctrl/lee_control.cpp
  src/est/imu_integ.cpp
  src/est/att_filter.cpp
  src/mixer/basic_mixer.cpp
  src/sysid/signal_gen.cpp
)

#############
# LIBRARIES #
#############
# NOTE(mereweth@jpl.nasa.gov) - Be careful when compiling Proto cc files. It is
# best to do so only once (make a shared library for each set of Proto files).
# Otherwise, at some point, you will get errors from double-adding protobuf
# formats, of the following form:
# [libprotobuf ERROR google/protobuf/descriptor_database.cc:57]
# File already exists in database: Block.proto

# Avoid having multiple compiled copies of the same .pb.cc
# file sharing a single copy of libprotobuf.so
#cs_add_library(${PROJECT_NAME}_proto
#  ${PROTO_SRCS}
#)
#target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

if (USE_ROS)
  cs_add_library(${PROJECT_NAME}
    ${${PROJECT_NAME}_SRCS}
  )
else()
  add_library(${PROJECT_NAME} SHARED
    ${${PROJECT_NAME}_SRCS}
  )
endif()
#target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

###################
# PYTHON BINDINGS #
###################
if(HAVE_PYBIND11)
  #catkin_python_setup()

  add_subdirectory(../tools/pybind11 pybind11)
  message("Building Python bindings for quest_gnc")
  pybind11_add_module(quest_gncpy
    ${${PROJECT_NAME}_SRCS}
    src/pybind11/rates_bind.cc
    src/pybind11/body_frame_bind.cc
    src/pybind11/lee_control_bind.cc
    src/pybind11/imu_integ_bind.cc
    src/pybind11/att_filter_bind.cc
    src/pybind11/signal_gen_bind.cc
    src/pybind11/common_bind.cc
    src/pybind11/bind.cc)
  set_target_properties(quest_gncpy PROPERTIES LINKER_LANGUAGE CXX)
  if (USE_ROS)
    target_link_libraries(quest_gncpy PUBLIC #${PROJECT_NAME}_proto
                                             #${PROTOBUF_LIBRARIES}
                                             ${catkin_LIBRARIES}
    )
  else()
    target_link_libraries(quest_gncpy PUBLIC #${PROJECT_NAME}_proto
                                             #${PROTOBUF_LIBRARIES}
    )
  endif()

  if (USE_ROS)
    set_target_properties(quest_gncpy
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/../)
  else()
  endif()
endif()

############
# BINARIES #
############

#add_executable(EXEC_NAME
#  SRC_FILE_NAME
#)
#target_link_libraries(EXEC_NAME ${PROJECT_NAME} ${catkin_LIBRARIES})

#########
# TESTS #
#########

if (USE_ROS)
  catkin_add_gtest(lee_control_phasing_test
    test/ctrl/lee_control_phasing_test.cpp)
  target_link_libraries(lee_control_phasing_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

##########
# EXPORT #
##########
if (USE_ROS)
  cs_install()
  cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
            #CFG_EXTRAS quest_gnc-extras.cmake
            LIBRARIES ${ADDITIONAL_LIBRARIES})
else()
  find_package( Eigen3 REQUIRED NO_MODULE )
  include_directories(include ${EIGEN3_INCLUDE_DIR})
endif()
