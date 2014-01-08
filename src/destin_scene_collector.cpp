/*
 * destin_scene_collector.cpp
 *
 *  Created on: Jan 8, 2014
 *      Author: Yuhuang Hu
 */

#include "destin_package/IncludedLibraries.hpp"
#include "destin_package/UtilityFunctions.hpp"

////// Global variables //////

// Constant
const int IMAGE_SIZE = 512;
const int FRAME_COUNT = 1000;
const string DESTIN_ORIGIN = "../result/destin_origin.des";
const string SAVED_DESTIN_FEATURES = "../result/saved_features/saved_destin_features.txt";

// Variables

int counter;
int destinCounter;
cv::Mat mainFrame;
ifstream fin;
ofstream fout;

// DeSTIN  network object

SupportedImageWidths siw = W512;
uint centroid_counts[] = {32, 32, 32, 32, 32, 32, 16, 10};
bool isUniform = true;
int nLayers = 8;
DestinNetworkAlt * network;
BeliefExporter * featureExtractor;

void destinSceneCollectorCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception & e)
  {
    cout << "error" << endl;
    return;
  }

  cv::Mat buffer = cv_ptr->image;
  mainFrame = processImage(buffer, IMAGE_SIZE);
  waitKey(3);
}

int main(int argc, char ** argv)
{
  // Initilization

  counter = 0;
  destinCounter = 0;

  ros::init(argc, argv, "destin_scene_collector");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/camera/rgb/image_raw", 1000, destinSceneCollectorCallBack);

  ros::Rate rate(5000);

  network = new DestinNetworkAlt(siw, nLayers, centroid_counts, isUniform);

  cout << "[MESSAGE] DeSTIN network created completed" << endl;

  //network->save((char *)DESTIN_ORIGIN.c_str());
  //return 0;

  // load original

  while (true)
  {
    ros::spinOnce();
    if (!mainFrame.data)
      continue;
    else
      break;
  }

  network->load(DESTIN_ORIGIN.c_str());
  for (int i = 0; i < nLayers; i++)
    network->setLayerIsTraining(i, true);

  while (ros::ok())
  {
    ros::spinOnce();

    if (counter <= FRAME_COUNT)
    {
      float * float_image = new float[IMAGE_SIZE * IMAGE_SIZE];
      callImage(mainFrame, IMAGE_SIZE, float_image);

      network->doDestin(float_image);

      delete float_image;

      cout << "[MESSAGE][TRAINING] Frame: " << counter << " is trained" << "\r";
      cout.flush();

      counter++;
    }
    else
    {
      cout << endl;
      // extract feature
      featureExtractor = new BeliefExporter(*network, 6);

      cout << "[MESSAGE] Feature extractor for DeSTIN network no. " << destinCounter << " is initialized" << endl;

      featureExtractor->writeBeliefToMat(SAVED_DESTIN_FEATURES);

      stringstream ss;
      string destinName = "";

      ss << "../result/destin_network/destin_" << destinCounter;
      ss >> destinName;

      network->save((char *)destinName.c_str());

      cout << "[MESSAGE] DeSTIN network no. " << destinCounter << " is saved" << endl;

      network->load(DESTIN_ORIGIN.c_str());

      for (int i = 0; i < nLayers; i++)
        network->setLayerIsTraining(i, true);

      counter = 0;
      destinCounter++;

      // Wait for 2 seconds

      cout << "[MESSAGE] PLEASE TURN YOUR ROBOT TO ANOTHER SCENE IN 5 SECONDS" << endl;
      rate.sleep();

    }

    imshow("DeSTIN Scene Collector", mainFrame);
  }
}

