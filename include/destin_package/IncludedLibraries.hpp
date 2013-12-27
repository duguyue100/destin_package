/*
 * IncludedLibraries.hpp
 *
 *  Created on: Dec 26, 2013
 *      Author: Yuhuang Hu
 */

#ifndef INCLUDEDLIBRARIES_HPP_
#define INCLUDEDLIBRARIES_HPP_

// ROS LIBRARY
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

// OPENCV LIBRARY
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// PCL LIBRARY

// DeSTIN LIBRARY

#include "DestinNetworkAlt.h"
#include "Transporter.h"
#include "unit_test.h"
#include "BeliefExporter.h"

// STANDARD LIBRARY
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cfloat>

// NAMING SPACE
using namespace std;
using namespace cv;

#endif /* INCLUDEDLIBRARIES_HPP_ */
