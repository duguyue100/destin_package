/*
 * UtilityFunctions.hpp
 *
 *  Created on: Dec 26, 2013
 *      Author: Yuhuang Hu
 */

#ifndef UTILITYFUNCTIONS_HPP_
#define UTILITYFUNCTIONS_HPP_

#include "destin_package/IncludedLibraries.hpp"

double calculateDistance(int featureDimension, float * selected_destin, float * testing_feature)
{
  double distance=0;

  for (int i=0;i<featureDimension;i++)
  {
    distance+=(selected_destin[i]-testing_feature[i])*(selected_destin[i]-testing_feature[i]);
  }

  distance=sqrt(distance)*10;

  return distance;
}

void convert(cv::Mat& in, float * out)
{
  if (in.channels() != 1)
  {
    throw runtime_error("Excepted a grayscale image with one channel.");
  }

  if (in.depth() != CV_8U)
  {
    throw runtime_error("Expected image to have bit depth of 8bits unsigned integers ( CV_8U )");
  }

  cv::Point p(0, 0);
  int i = 0;
  for (p.y = 0; p.y < in.rows; p.y++)
  {
    for (p.x = 0; p.x < in.cols; p.x++)
    {
      out[i] = (float)in.at<uchar>(p) / 255.0;
      i++;
    }
  }
}

int findMostSimilarDeSTINNetwork(int noDestin, int featureDimension, vector<float *> saved_feature, float * testing_feature)
{
  int result=-1;
  double distance=DBL_MAX;
  for (int i=0;i<noDestin;i++)
  {
    double d=calculateDistance(featureDimension, saved_feature[i], testing_feature);

    if (d<distance)
    {
      distance=d;
      result=i;
    }
  }

  return result;
}

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

void testNan(float * array, int len)
{
  for (int i=0; i<len; i++)
  {
    if (isnan(array[i]))
    {
      printf("input had nan\n");
      exit(1);
    }
  }
}

float * callImage(cv::Mat& image, int IMAGE_SIZE)
{
  float * float_image = new float[IMAGE_SIZE * IMAGE_SIZE];

  convert(image, float_image);

  testNan(float_image, IMAGE_SIZE * IMAGE_SIZE);

  return float_image;
}

#endif /* UTILITYFUNCTIONS_HPP_ */
