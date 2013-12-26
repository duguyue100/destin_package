A ROS-DeSTIN package
==============

## Introduction

This project is aiming to create a easy-to-use package for using DeSTIN in ROS with TurtleBot.

Current Nodes:

+ `grow_destin`: planning to create a growing DeSTIN with ROS.

## Updates and Development Plan

### Updates

+ 2012-12-26
  - The linking problem of DeSTIN is sloved.
  - The basic structure is established.

### Development Plan

+ Create a set of utility functions
+ Growing DeSTIN network
+ Make libraries linking more flexible
+ Integrate RGB channels

## Notes

+ You need to change the linking libraries part in the CMakeLists.txt, the format is as following
```
include_directories(/home/your_hostname/destin/Destin/Common)
include_directories(/home/your_hostname/destin/Destin/DavisDestin/include)
target_link_libraries(grow_destin /home/your_hostname/destin/Destin/Common/libcommon.so)
target_link_libraries(grow_destin /home/your_hostnamedestin/Destin/DavisDestin/libdestinalt.so)
```
Replace `your_hostname` to your own machine's username, then the building should be fine.

## Contact

Yuhuang Hu (duguyue100)

Advanced Robotic Lab,

Department of Artifical Intelligence,

Faculty of Computer Science & IT,

University of Malaya.

duguyue100@gmail.com
