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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(grow_destin src/grow_destin.cpp)
include_directories(/home/arlmaster/destin/Destin/Common)
include_directories(/home/arlmaster/destin/Destin/DavisDestin/include)
target_link_libraries(grow_destin /home/arlmaster/destin/Destin/Common/libcommon.so)
target_link_libraries(grow_destin /home/arlmaster/destin/Destin/DavisDestin/libdestinalt.so)

rosbuild_add_executable(destin_scene_collector src/destin_scene_collector.cpp)
include_directories(/home/arlmaster/destin/Destin/Common)
include_directories(/home/arlmaster/destin/Destin/DavisDestin/include)
target_link_libraries(destin_scene_collector /home/arlmaster/destin/Destin/Common/libcommon.so)
target_link_libraries(destin_scene_collector /home/arlmaster/destin/Destin/DavisDestin/libdestinalt.so)
