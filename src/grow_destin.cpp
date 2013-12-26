/*
 * grow_destin.cpp
 *
 *  Created on: Dec 26, 2013
 *      Author: Yuhuang Hu
 */

#include "destin_package/IncludedLibraries.hpp"
#include "destin_package/UtilityFunctions.hpp"

// Global variables

const int IMAGE_SIZE=256;


void growDestinCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    cout << "error" << endl;
    return ;
  }

  cv::Mat buffer=cv_ptr->image;

  cv::Mat frame=processImage(buffer, IMAGE_SIZE);

  imshow("Grow DeSTIN", buffer);

  waitKey(3);

}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "grow_destin");

  ros::NodeHandle node;

  ros::Subscriber sub=node.subscribe("/camera/rgb/image_raw", 1000, growDestinCallBack);

  ros::spin();

  return 0;
}
