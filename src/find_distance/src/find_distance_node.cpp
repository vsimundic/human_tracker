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

// PCL and PointCloud
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Aruco markers
// #include <opencv2/aruco.hpp>

#include <ros/console.h>

using namespace message_filters::sync_policies;
using namespace roi_msgs;
using namespace sensor_msgs;
using namespace sensor_msgs::image_encodings;
// using namespace cv::aruco;
using sensor_msgs::Image;
using cv_bridge::CvImagePtr;
using std::vector;
using std::string;
using cv::Rect;
using cv::Mat;
using roi_msgs::HumanEntries;
using roi_msgs::HumanEntry;
using roi_msgs::Rois;
using message_filters::TimeSynchronizer;
using std_msgs::Float64;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

// Structure for fixed point
struct fixedPt
{
  double x;
  double y;
  double z;
} pt;

// Declaration of publishers
ros::Publisher marker_pub;

// Definition of the function for creating point markers
Marker createFixedPt(double x, double y, double z)
{
  visualization_msgs::Marker markerPt;
  markerPt.id = 1;
  markerPt.header.frame_id = "/camera_link";
  markerPt.header.stamp = ros::Time::now();
  markerPt.ns = "point";

  markerPt.type = visualization_msgs::Marker::SPHERE;
  markerPt.action = visualization_msgs::Marker::ADD;

  markerPt.pose.position.x = x;
  markerPt.pose.position.y = y;
  markerPt.pose.position.z = z;
  markerPt.pose.orientation.x = 0.0;
  markerPt.pose.orientation.y = 0.0;
  markerPt.pose.orientation.z = 0.0;
  markerPt.pose.orientation.w = 1.0;
  markerPt.scale.x = 0.2;
  markerPt.scale.y = 0.2;
  markerPt.scale.z = 0.2;
  markerPt.color.a = 1.0;  // Don't forget to set the alpha!
  markerPt.color.r = 1.0;
  markerPt.color.g = 1.0;
  markerPt.color.b = 0.0;
  markerPt.lifetime = ros::Duration();

  return markerPt;
}

// Callback function
void distanceCallback(const HumanEntriesConstPtr& entries_msg)
{
  // Distance flag
  bool distance_flag = false;

  // Define markers for publishing
  MarkerArray markers;
  markers.markers.clear();

  for (int i = 0; i < entries_msg->entries.size(); i++)
  {
    // Get the entry from the received message
    HumanEntry entry = entries_msg->entries[i];

    double x = entry.personCentroidY;
    double y = -entry.personCentroidX;
    double z = -entry.personCentroidZ;

    // Calcualte Euclidean distance between fixed point and human's head
    double entryDistance = sqrt(pow(pt.x - x, 2) + pow(pt.y - y, 2));

    // ROS_INFO("Distance to point: %f", entryDistance);

    if (entryDistance < 1.5)
    {
      distance_flag = true;
      break;
    }
  }

  // Output an appropriate message based on the calculated distance
  /*
  if (distance_flag)
    ROS_ERROR("It's too close!");
  else
    ROS_WARN("It's far enough!");

  */
  // Publish to topics
  markers.markers.push_back(createFixedPt(pt.x, pt.y, pt.z));
  marker_pub.publish(markers);
}

int main(int argc, char** argv)
{
  // Initialization of the node
  ros::init(argc, argv, "FindDistance");
  ros::NodeHandle n;

  // Define the fixed pt
  pt.x = 2.0;    // in method this is y
  pt.y = 1.0;    // in method this is minus x
  pt.z = -0.75;  // in method this is minus z

  // Create and save ArUco marker
  // Mat image;
  // cv::Ptr<Dictionary> dict = getPredefinedDictionary(DICT_6X6_250);
  // drawMarker(dict, 23, 200, image, 1);
  // cv::imwrite("~/human_tracker_ws/marker23.png", image);

  // Standard subs and pubs
  marker_pub = n.advertise<MarkerArray>("/fixedPt", 2);
  message_filters::Subscriber<HumanEntries> entry_sub(n, "human_tracker_data", 2);
  entry_sub.registerCallback(distanceCallback);

  // Tell ROS to loop the callback function in the node
  ros::spin();

  return 0;
}