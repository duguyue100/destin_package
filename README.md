A ROS-DeSTIN package
==============

## Introduction

This project is aiming to create a easy-to-use package for using DeSTIN in ROS with TurtleBot.

Current Nodes:

+ `grow_destin`: planning to create a growing DeSTIN with ROS.

## Updates and Development Plan

### Updates

+ 2013-12-26
  - The linking problem of DeSTIN is sloved.
  - The basic structure is established.
  - 2 collection of libraries are added.
+ 2013-12-27
  - First roughly working version is updated.
  - Some bugs are fixed
+ 2014-01-08
  - DeSTIN Scene Collector is updated.

### Development Plan

+ Create a set of utility functions [DONE, STILL IMPROVING]
+ Growing DeSTIN network [FIRST VERSION COMPLETED, TESTED]
+ Add condition to deal with initial situation [DONE]
+ Add condition to deal with when the selected DeSTIN network is updated (now everytime will create new DeSTIN network) [DONE]
+ Add some messages to control the flow [DONE]
+ Add a pause [DONE]
+ The numbering of extracted features and DeSTIN network is not same, should be fixed [NOT SO SURE, NEED A CLOSER LOOK]
+ Change a distance measurement method by using Gaussian-like distribution.
+ Add a full log when the program is crushed so the program can be recovered. (and some functions to utilize this)
+ Testing Growing DeSTIN network with slow moving speed [DONE]
+ Memory monitoring [DONE]
+ Test if the signal transmit is working well; if not, find out a way to control the speed.
+ Make libraries linking more flexible
+ Integrate RGB channels
+ Change `sleep()` function for DeSTIN Scene Collector

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
+ In ROS, the `ros::spinOnce()` function is useful when you want to drag all the data from subscribed topic to your main program. And in this package, we use a global variable to copy out the data and then process it in a endless loop in main.
+ When `network->load(filename)` is executed, the layers of training are automatically closed. You need to open them manually. [IMPROTANT]

+ DeSTIN Scene Collector is a simple DeSTIN network collector for different scenes. Each DeSTIN network is trained for 1000 frames with 512*512 gray scale image. After the scene is trained, the user has enough time to switch to another scene for training next DeSTIN network.

## Contact

Yuhuang Hu (duguyue100)

Advanced Robotic Lab,

Department of Artifical Intelligence,

Faculty of Computer Science & IT,

University of Malaya.

duguyue100@gmail.com
