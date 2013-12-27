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
const int IMAGE_SIZE = 256;
const int FRAME_COUNT = 1000;
const int TESTING_FRAME_COUNT = 400;
const int TRAINING_STATE_1 = 1;
const int TRAINING_STATE_2 = 2;
const int TRAINING_STATE_3 = 3;
const int TRAINING_STATE_4 = 4;
const int TESTING_STATE_ON = 1;
const int TESTING_STATE_OFF = 2;
const double DESTIN_DISTANCE = 2.0;
const string DESTIN_ORIGIN = "../result/destin_origin.des";
const string SAVED_DESTIN_FEATURES = "../result/saved_features/saved_destin_features.txt";

// Variables

int counter;
int testing_counter;
int noDestin;
int training_state;
int testing_state;
int selected_destin;
float * testingFeature;
vector<cv::Mat> saved_frame;
cv::Mat mainFrame;
ifstream fin;
ofstream fout;

// DeSTIN network object

SupportedImageWidths siw = W256;
uint centroid_counts[] = {32, 32, 32, 32, 32, 16, 10};
bool isUniform = true;
int nLayers = 7;
DestinNetworkAlt * network;
BeliefExporter * featureExtractor;

void growDestinCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
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
  noDestin = 0;
  selected_destin = -1;
  counter = 0;
  testing_counter = 0;
  testing_state = TESTING_STATE_OFF;
  training_state = TRAINING_STATE_2;

  //saved_frame.clear();

  cout << "[MESSAGE] Main parameters are initialized" << endl;

  ros::init(argc, argv, "grow_destin");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/camera/rgb/image_raw", 1000, growDestinCallBack);

  ros::Rate rate(20);

  while (true)
  {
    ros::spinOnce();
    if (!mainFrame.data)
      continue;
    else
      break;
  }

  cout << "[MESSAGE] DATA IS START TO RECEIVE" << endl;

  network = new DestinNetworkAlt(siw, nLayers, centroid_counts, isUniform);

  cout << "[MESSAGE] New DeSTIN network is generated" << endl;

  while (true)
  {
    ros::spinOnce();

    if (counter == FRAME_COUNT && testing_state == TESTING_STATE_OFF)
    {
      counter = 0;
      featureExtractor = new BeliefExporter(*network, 5);
      cout << "[MESSAGE] Feature extractor initialization completed" << endl;

      stringstream ss;
      string destin_name = "";

      if (selected_destin == -1)
      {
        ss << "../result/destin_network/destin_" << noDestin;
        ss >> destin_name;
      }
      else
      {
        ss << "../result/destin_network/destin_" << selected_destin;
        ss >> destin_name;
      }

      network->save((char *)destin_name.c_str());

      cout << "[MESSAGE] DeSTIN network " << noDestin << " is saved" << endl;

      // disable training

      //    for (int i = 0; i < nLayers; i++)
      //      network->setLayerIsTraining(i, false);
      //
      //    network->clearBeliefs();

      // extract features

      //    int savedImageLength = saved_frame.size();
      //
      //    for (int i = 0; i < savedImageLength; i++)
      //    {
      //      float * float_image = callImage(saved_frame[i], IMAGE_SIZE);
      //      network->doDestin(float_image);
      //    }

      if (selected_destin == -1)
        featureExtractor->writeBeliefToMat(SAVED_DESTIN_FEATURES);

      for (int i = 0; i < nLayers; i++)
        network->setLayerIsTraining(i, false);

      network->clearBeliefs();

      cout << "[MESSAGE] Feature of DeSTIN network " << noDestin << " is extracted" << endl;
      // clear DeSTIN

      //saved_frame.clear();
      if (selected_destin == -1)
        noDestin++;
      testing_state = TESTING_STATE_ON;
    }
    else if (testing_counter <= 200 && testing_state == TESTING_STATE_ON)
    {
      rate.sleep();
      testing_counter++;
      cout << "[MESSAGE][TESTING] FRAME: " << testing_counter << " is dumped" << "\r";
    }
    else if (testing_counter > 200 && testing_counter < TESTING_FRAME_COUNT && testing_state == TESTING_STATE_ON)
    {
      float * float_image = callImage(mainFrame, IMAGE_SIZE);
      network->doDestin(float_image);

      cout << "[MESSAGE][TESTING] Frame: " << testing_counter << " is trained" << "\r";
      cout.flush();

      testing_counter++;
      if (testing_counter == TESTING_FRAME_COUNT)
      {
        testing_state = TESTING_STATE_OFF;
        training_state = TRAINING_STATE_1;
        //BeliefExporter * featureExtractor = new BeliefExporter(*network, 5);
        //float * testingFeature = featureExtractor->getBeliefs();
      }
    }
    else if (counter < FRAME_COUNT && testing_state == TESTING_STATE_OFF)
    {

      switch (training_state)
      {
        case TRAINING_STATE_1: // search for existing DeSTIN network
        {
          if (noDestin == 0)
          {
            training_state = TRAINING_STATE_2;
            break;
          }
          testing_counter = 0;
          float * testing_feature = featureExtractor->getBeliefs();
          fin.open(SAVED_DESTIN_FEATURES.c_str());

          int outputSize = featureExtractor->getOutputSize();
          vector<float *> saved_feature;
          for (int i = 0; i < noDestin; i++)
          {
            float * temp = new float[outputSize];

            for (int j = 0; j < outputSize; j++)
              fin >> temp[j];

            saved_feature.push_back(temp);
          }

          fin.close();

          selected_destin = findMostSimilarDeSTINNetwork(noDestin, outputSize, saved_feature, testing_feature);
          double distance = calculateDistance(outputSize, saved_feature[selected_destin], testing_feature);

          cout << "[MESSAGE] DeSTIN network No. " << selected_destin << " is the most similar network" << endl;
          cout << "[MESSAGE] The distance is " << distance << endl;

          if (distance >= DESTIN_DISTANCE)
          {
            training_state = TRAINING_STATE_2;
          }
          else
          {
            training_state = TRAINING_STATE_3;
          }

          break;
        }
        case TRAINING_STATE_2: // if need to create new DeSTIN network
        {
          selected_destin = -1;
          network->load(DESTIN_ORIGIN.c_str());

          for (int i = 0; i < nLayers; i++)
            network->setLayerIsTraining(i, true);

          cout << "[MESSAGE] New DeSTIN network is generated" << endl;

          training_state = TRAINING_STATE_4;
          break;
        }
        case TRAINING_STATE_3: // if need to feedback from old DeSTIN
        {
          string selected_destin_name = "";

          stringstream ss;
          ss << "../result/destin_network/destin_" << selected_destin;
          ss >> selected_destin_name;

          network->load(selected_destin_name.c_str());

          for (int i = 0; i < nLayers; i++)
            network->setLayerIsTraining(i, true);

          cout << "[MESSAGE] DeSTIN network No. " << selected_destin << " is used" << endl;

          training_state = TRAINING_STATE_4;
          break;
        }
        case TRAINING_STATE_4:
        {
          //saved_frame.push_back(mainFrame);

          float * float_image = callImage(mainFrame, IMAGE_SIZE);

          network->doDestin(float_image);

          //network->printBeliefGraph(6, 0, 0);

          cout << "[MESSAGE][TRAINING] Frame: " << counter << " is trained" << "\r";
          cout.flush();

          counter++;
          break;
        }
        default:
        {
          break;
        }
      }
    }

    imshow("Grow DeSTIN", mainFrame);
  }

  return 0;
}
