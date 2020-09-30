#include "dataset_testing/dataset_testing.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include "ros/ros.h"

#include <std_msgs/Float64.h>
#include "roi_msgs/HumanEntries.h"
#include "roi_msgs/HumanEntry.h"
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"

// Time Synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

// Subscribe Messages
// #include </home/valentin/cv_bridge/include/cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/console.h>

using namespace message_filters::sync_policies;
using namespace roi_msgs;
using namespace sensor_msgs;
using sensor_msgs::Image;
using cv_bridge::CvImagePtr;
using roi_msgs::HumanEntries;
using message_filters::TimeSynchronizer;

// Logger
std::ofstream log_file;
std::string path_to_file = "/home/valentin/human_tracker_ws/FERIT_dataset/kinect_k31_1/annot_images/";

// Structure for fixed point
struct fixedPt
{
  double x;
  double y;
  double z;
} pt;

void entryImageCallback(const HumanEntriesConstPtr entries_msg)
{
  std::ostringstream str_stream;
  str_stream << std::setprecision(17) << entries_msg->header.stamp.toSec();
  std::string strStamp = str_stream.str();

  
  int numEntriesInRange = 0;

  for (int i = 0; i < entries_msg->entries.size(); i++)
  {
    HumanEntry entry = entries_msg->entries[i];

    double x = entry.personCentroidY;
    double y = -entry.personCentroidX;
    double calcd_distance = sqrt(pow(pt.x - x, 2) + pow(pt.y - y, 2));
    // ROS_WARN("Distance[%d]: %f", i, calcd_distance);

    if (calcd_distance < 2.0)
      numEntriesInRange++;
  }
  
  log_file << entries_msg->entries.size() << "," << strStamp << std::endl;
  log_file << entries_msg->entries.size() << "," << strStamp << "," << numEntriesInRange << std::endl;
}

void onlyImageCallback(const ImageConstPtr image_msg)
{

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

  // sensor_msgs::CvBridge bridge;
  // IplImage* ipl_im = bridge.imgMsgToCv(image_msg, "passthrough");

  std::ostringstream str_stream;
  str_stream << std::setprecision(17) << image_msg->header.stamp.toSec();
  std::string strStamp = str_stream.str();

  log_file << 0 << "," << strStamp << std::endl;
  std::string image_name = "/home/valentin/human_tracker_ws/FERIT_dataset/kinect_k07_1/images/" + strStamp + ".pn"
                                                                                                             "g";
  std::cout << std::setprecision(17) << image_msg->header.stamp.toSec() << std::endl;

  cv::imwrite(image_name.c_str(), cv_ptr->image);
}

int main(int argc, char** argv)
{
  // Initialization of the node
  ros::init(argc, argv, "DatasetTesting");
  ros::NodeHandle n;

  // Define the fixed pt
  pt.x = 2.95;   // in method this is y
  pt.y = 1.6;    // in method this is minus x
  pt.z = -0.25;  // in method this is minus z

  // Logger for entries data
  log_file.open("/home/valentin/human_tracker_ws/FERIT_dataset/kinect_k07_1/results/k07_method21.csv",
                std::ios_base::out);
  log_file << "PersoneNumber"
           << ","
           << "TIME"
           << "," << std::endl;

  // Simple subscriber for getting only image
  // ros::Subscriber image_sub = n.subscribe("/camera/rgb/image_color", 1030, onlyImageCallback);

  ros::Subscriber image_sub = n.subscribe("human_tracker_data", 1, entryImageCallback);
  // Tell ROS to loop the callback function in the node
  ros::spin();

  return 0;
}
