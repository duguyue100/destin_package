/*
 * grow_destin.cpp
 *
 *  Created on: Dec 26, 2013
 *      Author: Yuhuang Hu
 */

#include "destin_package/IncludedLibraries.hpp"
#include "destin_package/UtilityFunctions.hpp"

////// Global variables //////

// Constant
const int IMAGE_SIZE=256;
const int FRAME_COUNT=1000;
const int TESTING_FRAME_COUNT=50;
const int TRAINING_STATE_1=1;
const int TRAINING_STATE_2=2;
const int TRAINING_STATE_3=3;
const int TRAINING_STATE_4=4;
const int TESTING_STATE_ON=1;
const int TESTING_STATE_OFF=2;
const double DESTIN_DISTANCE=35;
const string DESTIN_ORIGIN="../result/destin_origin.des";

// Variables

int counter;
int testing_counter;
int noDestin;
int training_state;
int testing_state;
string selected_destin_name;
vector<cv::Mat> saved_frame;
ifstream fin;
ofstream fout;

// DeSTIN network object

SupportedImageWidths siw=W256;
uint centroid_counts[]={32,32,32,32,32,16,10};
bool isUniform=true;
int nLayers=7;
DestinNetworkAlt * network;

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

  cout << "[MESSAGE] Frame: " << counter << "\r";
  cout.flush();

  // Will the data drag?

  if (counter==FRAME_COUNT && testing_state==TESTING_STATE_OFF)
  {
    counter=0;
    BeliefExporter * featureExtractor=new BeliefExporter(*network, 5);
    cout << "[MESSAGE] Feature extractor initialization completed" << endl;

    stringstream ss;
    string destin_name="";
    ss << "../result/destin_network/destin_" << noDestin;
    ss >> destin_name;

    network->save((char *)destin_name.c_str());

    cout << "[MESSAGE] DeSTIN network " << noDestin << " is saved" << endl;

    // disable training

    for (int i=0;i<nLayers;i++) network->setLayerIsTraining(i, false);

    network->clearBeliefs();

    // extract features

    int savedImageLength=saved_frame.size();

    for (int i=0;i<savedImageLength;i++)
    {
      float * float_image=callImage(saved_frame[i], IMAGE_SIZE);
      network->doDestin(float_image);
    }

    featureExtractor->writeBeliefToMat("../result/saved_features/saved_destin_features.txt");

    cout << "[MESSAGE] Feature of DeSTIN network " << noDestin << " is extracted" << endl;
    // clear DeSTIN

    saved_frame.clear();
    noDestin++;
    testing_state=TESTING_STATE_ON;
  }
  else if (testing_counter==TESTING_FRAME_COUNT && testing_state==TESTING_STATE_ON)
  {
    testing_counter=0;

    float * float_image=callImage(frame, IMAGE_SIZE);
    network->doDestin(float_image);

    testing_state=TESTING_STATE_OFF;

  }
  else if (counter < FRAME_COUNT && testing_state==TESTING_STATE_OFF)
  {
    saved_frame.push_back(frame);

    switch (training_state)
    {
      case TRAINING_STATE_1: // search for existing DeSTIN network
      {
        selected_destin_name=findMostSimilarDeSTINNetwork();
        int distance=calculateDistance();

        if (distance>=DESTIN_DISTANCE)
        {
          training_state=TRAINING_STATE_2;
        }
        else
        {
          training_state=TRAINING_STATE_3;
        }

        break;
      }
      case TRAINING_STATE_2: // if need to create new DeSTIN network
      {
        network=new DestinNetworkAlt(siw, nLayers, centroid_counts, isUniform);
        network->load(DESTIN_ORIGIN.c_str());

        training_state=TRAINING_STATE_4;
        break;
      }
      case TRAINING_STATE_3: // if need to feedback from old DeSTIN
      {
        network=new DestinNetworkAlt(siw, nLayers, centroid_counts, isUniform);
        network->load(selected_destin_name.c_str());

        training_state=TRAINING_STATE_4;
        break;
      }
      case TRAINING_STATE_4:
      {
        float * float_image=callImage(frame, IMAGE_SIZE);

        network->doDestin(float_image);

        break;
      }
      default:
      {
        break;
      }
    }

    counter++;
  }

  imshow("Grow DeSTIN", frame);

  waitKey(3);

}

int main(int argc, char ** argv)
{
  noDestin=0;
  counter=0;
  testing_counter=0;
  testing_state=TESTING_STATE_OFF;
  saved_frame.clear();

  ros::init(argc, argv, "grow_destin");

  ros::NodeHandle node;

  ros::Subscriber sub=node.subscribe("/camera/rgb/image_raw", 1000, growDestinCallBack);

  ros::spin();

  return 0;
}
