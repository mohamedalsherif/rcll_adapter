cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


set(CPP11_EXTRA_FLAGS "")
list(APPEND CMAKE_CXX_FLAGS "-std=c++11 -pthread  ${CPP11_EXTRA_FLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "-pthread ${CMAKE_EXE_LINKER_FLAGS} ${CPP11_EXTRA_FLAGS}")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Fawkes COMPONENTS core blackboard interface navgraph protobuf_comm SkillerInterface)
if (Fawkes_FOUND)
  add_library(FawkesBackend SHARED
    src/FawkesHandler.cpp)

  # This is necessary due to some hacks regarding LLSF processing
  # in the FawkesBackend. This should go away once YAGI message
  # processing has been upgrade.
  list(APPEND Fawkes_LIBRARIES "llsf_msgs")

  set_target_properties(FawkesBackend PROPERTIES
    INCLUDE_DIRECTORIES "${Fawkes_INCLUDE_DIRS}"
    COMPILE_FLAGS "${Fawkes_CFLAGS}"
    LINK_FLAGS "${Fawkes_LFLAGS}"
    LINK_LIBRARIES "${Fawkes_LIBRARIES}"
    INSTALL_RPATH "${Fawkes_LIBRARY_DIRS}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE)
  message(STATUS "Building Fawkes backend plugin")

else()
  message(STATUS "Fawkes not found, cannot build Fawkes plugin")
endif()

#  rosbuild_add_executable(talker src/talker.cpp)
#  rosbuild_add_executable(listener src/listener.cpp)
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_library(helloDemo SHARED "${Fawkes_LIBRARY_DIRS}")
#include_directories(${Fawkes_LIBRARY_DIRS})	
add_executable (helloDemo src/ros_rcll_starter.cpp)
rosbuild_add_executable(talker src/talker.cpp)
rosbuild_add_executable(listener src/listener.cpp)

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(GDKPIXBUF REQUIRED gdk-pixbuf-2.0)
#include_directories(${GDKPIXBUF_INCLUDE_DIRS})
#link_directories(${GDKPIXBUF_LIBRARY_DIRS})
#target_link_libraries(map_server ${GDKPIXBUF_LIBRARIES})
#target_link_libraries(listener Fawkes)


