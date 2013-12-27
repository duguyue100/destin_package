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
  - 2 collection of libraries are added.

### Development Plan

+ Create a set of utility functions [DONE, STILL IMPROVING]
+ Growing DeSTIN network [FIRST VERSION COMPLETED, HAVEN'T TESTED]
+ Add condition to deal with initial situation [DONE, HAVEN'T TESTED]
+ Add condition to deal with when the selected DeSTIN network is updated (now everytime will create new DeSTIN network) [DONE, HAVEN'T TESTED]
+ Add some messages to control the flow
+ Add a pause
+ Add a full log when the program is crushed so the program can be recovered. (and some functions to utilize this)
+ Testing Growing DeSTIN network with slow moving speed
+ Memory monitoring
+ Test if the signal transmit is working well; if not, find out a way to control the speed.
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

+ The basic idea of this program is as following (first draft)
  - The robot is trained by DeSTIN for 1000 frames in one scene.
  - Then the robot close and save the current configuration and then turn to a new scene
  - If the new scene is similar to saved DeSTIN networks enough, then will select a best fit network to update.
  - Otherwise will create a new DeSTIN network.

## Contact

Yuhuang Hu (duguyue100)

Advanced Robotic Lab,

Department of Artifical Intelligence,

Faculty of Computer Science & IT,

University of Malaya.

duguyue100@gmail.com
