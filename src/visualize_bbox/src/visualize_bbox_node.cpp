#include <sstream>
#include "ros/ros.h"

#include <std_msgs/Float64.h>

// Custom messages
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
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

ros::Publisher ptcloud_pub;
ros::Publisher markers_pub;

typedef ApproximateTime<PointCloud2, HumanEntries> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
boost::shared_ptr<ApproximateSync> approximate_sync_;

void entryCallback(const PointCloud2ConstPtr& cloud_msg, const HumanEntriesConstPtr& entry_msg)
{
  // Create array for markers
  visualization_msgs::MarkerArray pplMarkers;
  pplMarkers.markers.clear();

  ROS_ERROR("Marker count: %d", entry_msg->entries.size());

  // Loop through message data, create bbox markers and push on back of array
  for (int i = 0; i < entry_msg->entries.size(); i++)
  {
    visualization_msgs::Marker personMarker;
    personMarker.id = i;
    personMarker.header.frame_id = "/world";
    personMarker.header.stamp = ros::Time::now();

    personMarker.ns = "person";

    personMarker.type = visualization_msgs::Marker::CUBE;
    personMarker.action = visualization_msgs::Marker::ADD;

    personMarker.pose.position.x = entry_msg->entries[i].personBoundingBoxTopCenterY;
    personMarker.pose.position.y = -entry_msg->entries[i].personBoundingBoxTopCenterX;
    personMarker.pose.position.z = -entry_msg->entries[i].personBoundingBoxTopCenterZ;

    personMarker.pose.orientation.x = 0.0;
    personMarker.pose.orientation.y = 0.0;
    personMarker.pose.orientation.z = 0.0;
    personMarker.pose.orientation.w = 1.0;

    personMarker.scale.x = entry_msg->entries[i].ROIwidth;
    personMarker.scale.y = entry_msg->entries[i].ROIwidth;
    personMarker.scale.z = entry_msg->entries[i].ROIheight;

    personMarker.color.a = 0.28;  // Don't forget to set the alpha!
    personMarker.color.r = 0.0;
    personMarker.color.g = 1.0;
    personMarker.color.b = 0.0;

    personMarker.lifetime = ros::Duration();

    pplMarkers.markers.push_back(personMarker);
  }

  // Publish data
  markers_pub.publish(pplMarkers);
  ptcloud_pub.publish(cloud_msg);
}

int main(int argc, char** argv)
{
  // Initialization of the node
  ros::init(argc, argv, "VisualizeBbox");
  ros::NodeHandle n;

  // Check if visualization is on
  bool visualize_flag = false;
  n.param("/visualization/ptcloud", visualize_flag, false);

  if (!visualize_flag)
    n.shutdown();

  // Standard subs
  message_filters::Subscriber<PointCloud2> cloud_sub(n, "PtCloud", 2);
  message_filters::Subscriber<HumanEntries> entry_sub(n, "HumanData", 2);

  // Define synchronizer
  // TimeSynchronizer<PointCloud2, HumanEntries> sync(cloud_sub, entry_sub, 10);

  // Binding the callback function
  // sync.registerCallback(boost::bind(&entryCallback, _1, _2));

  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(4), cloud_sub, entry_sub));
  // register the 2 element callback
  approximate_sync_->registerCallback(boost::bind(&entryCallback, _1, _2));

  // publish Pointcloud and markers
  markers_pub = n.advertise<visualization_msgs::MarkerArray>("markersHumans", 2);
  ptcloud_pub = n.advertise<PointCloud2>("VisualizePtCloud", 2);

  // Tell ROS to loop the callback function in the node
  ros::spin();

  return 0;
}