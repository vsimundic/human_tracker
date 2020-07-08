#include "bbox_info/bbox_info.hpp"

#include <sstream>
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
#include </home/valentin/cv_bridge/include/cv_bridge/CvBridge.h>
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

// PCL and PointCloud
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/console.h>

using namespace message_filters::sync_policies;
using namespace roi_msgs;
using namespace sensor_msgs;
using namespace sensor_msgs::image_encodings;
using namespace stereo_msgs;
using sensor_msgs::Image;
using cv_bridge::CvImagePtr;
using std::vector;
using std::string;
using cv::Rect;
using cv::Mat;
using roi_msgs::HumanEntries;
using roi_msgs::Rois;
using message_filters::TimeSynchronizer;
using std_msgs::Float64;

// Callback function
void entryCallback(const ImageConstPtr& image_msg, const HumanEntriesConstPtr& entry_msg, const RoisConstPtr& rois_msg)
{
  // Changing the format from ROS message to cv image
  sensor_msgs::CvBridge bridge;
  IplImage* ipl_im = bridge.imgMsgToCv(image_msg, "passthrough");
  cv::Mat cv_color = cv::Mat(ipl_im).clone();

  // Loop through ROIs and create rectangles on the image
  for (int i = 0; i < rois_msg->rois.size(); i++)
  {
    roi_msgs::RoiRect r = rois_msg->rois[i];
    cv::Rect rectHuman(r.x, r.y, r.width, r.height);

    cv::rectangle(cv_color, rectHuman, cv::Scalar(0, 255, 0), 2);
  }

  // Show the image
  imshow("BboxInfo color", cv_color);
  cv::waitKey(1);
}

int main(int argc, char** argv)
{
  // Initialization of the node
  ros::init(argc, argv, "BboxInfo");
  ros::NodeHandle n;

  bool show_images = true;
  string node_name = ros::this_node::getName();
  n.param(node_name + "/Show_Images", show_images, true);

  if (!show_images)
  {
    ros::shutdown();
  }

  // Defining two subscribers, one for image and one for bounding box data
  // Standard stuff and subs
  message_filters::Subscriber<Image> image_sub(n, "HogSvmColorImage", 2);
  message_filters::Subscriber<HumanEntries> entry_sub(n, "human_tracker_data", 2);
  message_filters::Subscriber<Rois> rois_sub(n, "ObjectTrackingRois", 2);

  // Defining Synchronizer
  TimeSynchronizer<Image, HumanEntries, Rois> sync(image_sub, entry_sub, rois_sub, 10);

  // Creating a window to display the image
  cv::namedWindow("BboxInfo color", 0);

  // Declaration of the callback function
  sync.registerCallback(boost::bind(&entryCallback, _1, _2, _3));

  // Tell ROS to loop the callback function in the node
  ros::spin();

  return 0;
}