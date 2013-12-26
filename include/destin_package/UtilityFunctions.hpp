/*
 * UtilityFunctions.hpp
 *
 *  Created on: Dec 26, 2013
 *      Author: Yuhuang Hu
 */

#ifndef UTILITYFUNCTIONS_HPP_
#define UTILITYFUNCTIONS_HPP_

#include "destin_package/IncludedLibraries.hpp"

cv::Mat processImage(cv::Mat buffer, int image_size)
{
  cv::Mat image;

  cv::resize(buffer, buffer, Size(image_size, image_size), 0, 0, INTER_LINEAR);

  if (!buffer.data)
  {
    cout << "No data from image source" << endl;
  }

  cv::cvtColor(buffer, image, CV_BGR2GRAY);

  return image;

}

#endif /* UTILITYFUNCTIONS_HPP_ */
