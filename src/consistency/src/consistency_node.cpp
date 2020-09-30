/*
	Software License Agreement (BSD License)

	Copyright (c) 2013, Southwest Research Institute
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.
	* Neither the name of the Southwest Research Institute, nor the names
	of its contributors may be used to endorse or promote products derived
	from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>

// Ros package includes
#include <roi_msgs/RoiRect.h>
#include <roi_msgs/Rois.h>
#include <roi_msgs/overlap.hpp>
//#include </home/valentin/human_tracker/src/roi_msgs/include/roi_msgs/overlap.hpp>
#include <consistency/consistency.hpp>

// Time Synchronizer
// NOTE: Time Synchronizer conflicts with QT includes may need to investigate
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

// Subscribe Messages
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
// #include <cv_bridge/cvBridge.h>
// #include </home/valentin/human_tracker_ws/src/cv_bridge/include/cv_bridge/cv_bridge.h>
// #include </home/valentin/cv_bridge/include/cv_bridge/CvBridge.h>

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Includes for pointcloud
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace sensor_msgs::image_encodings;
using namespace roi_msgs;
using std::string;
using Cten::Consistency;
using namespace cv_bridge;
using sensor_msgs::PointCloud2;

class consistencyNode
{
private:
  // Define Node
  ros::NodeHandle node_;

  // Subscribe to Messages
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  message_filters::Subscriber<Image> sub_image_;
  message_filters::Subscriber<Rois> sub_rois_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_ptcloud_;

  // Define the Synchronizer
  typedef ApproximateTime<Image, DisparityImage, Rois> ApproximatePolicy;
  typedef ApproximateTime<Image, DisparityImage> ApproximatePolicy2;
  typedef ApproximateTime<Image, DisparityImage, Rois, PointCloud2> ApproximatePolicyWithPtcloud;
  typedef ApproximateTime<Image, DisparityImage, PointCloud2> ApproximatePolicy2WithPtcloud;

  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  typedef message_filters::Synchronizer<ApproximatePolicy2> ApproximateSync2;
  typedef message_filters::Synchronizer<ApproximatePolicyWithPtcloud> ApproximateSyncWithPtcloud;
  typedef message_filters::Synchronizer<ApproximatePolicy2WithPtcloud> ApproximateSync2WithPtcloud;

  boost::shared_ptr<ApproximateSync> approximate_sync_;
  boost::shared_ptr<ApproximateSync2> approximate_sync2_;
  boost::shared_ptr<ApproximateSyncWithPtcloud> approximate_sync_with_ptcloud_;
  boost::shared_ptr<ApproximateSync2WithPtcloud> approximate_sync2_with_ptcloud_;

  // Messages to Publish
  ros::Publisher pub_rois_;
  ros::Publisher pub_Color_Image_;
  ros::Publisher pub_Disparity_Image_;
  ros::Publisher pub_Ptcloud;

  Rois output_rois_;

  // Define the Classifier Object
  Consistency con_;

  // Mode of operation
  string mode;
  bool useDefaultRoi;

  // Use visualization
  bool visualize_flag;

public:
  explicit consistencyNode(const ros::NodeHandle& nh) : node_(nh)
  {
	string nn = ros::this_node::getName();
	int qs;
	if (!node_.getParam(nn + "/Q_Size", qs))
	{
	  qs = 3;
	}

	// Published Messages
	pub_rois_ = node_.advertise<Rois>("ConsistencyOutputRois", qs);
	pub_Color_Image_ = node_.advertise<Image>("ConsistencyColorImage", qs);
	pub_Disparity_Image_ = node_.advertise<DisparityImage>("ConsistencyDisparityImage", qs);

	// Subscribe to Messages
	sub_image_.subscribe(node_, "Color_Image", qs);
	sub_disparity_.subscribe(node_, "Disparity_Image", qs);

	// Check visualization param
	node_.param("/visualization/ptcloud", visualize_flag, false);
	if (visualize_flag)
	{
	  pub_Ptcloud = node_.advertise<PointCloud2>("ConsistencyPtcloud", qs);
	  sub_ptcloud_.subscribe(node_, "PtCloud", qs);
	}

	if (!node_.getParam(nn + "/UseDefaultRois", useDefaultRoi))
	{
	  useDefaultRoi = false;
	}
	if (useDefaultRoi)
	{  // no roi available, use default
	  ROS_ERROR("Using default roi");

	  if (!visualize_flag)
	  {
		// sync the syncronizer
		approximate_sync2_.reset(new ApproximateSync2(ApproximatePolicy2(qs), sub_image_, sub_disparity_));
		// register the 2 element callback
		approximate_sync2_->registerCallback(boost::bind(&consistencyNode::imageCb2, this, _1, _2));
	  }
	  else
	  {
		approximate_sync2_with_ptcloud_.reset(new ApproximateSync2WithPtcloud(
			ApproximatePolicy2WithPtcloud(qs), sub_image_, sub_disparity_, sub_ptcloud_));
		approximate_sync2_with_ptcloud_->registerCallback(
			boost::bind(&consistencyNode::realCallback2WithPtcloud, this, _1, _2, _3));
	  }
	}
	else
	{
	  // subscribe to the rois
	  sub_rois_.subscribe(node_, "input_rois", 1);

	  if (!visualize_flag)
	  {
		// Sync the Synchronizer
		approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(4), sub_image_, sub_disparity_, sub_rois_));

		// register the 3 element callback
		approximate_sync_->registerCallback(boost::bind(&consistencyNode::imageCb, this, _1, _2, _3));
	  }
	  else
	  {
		approximate_sync_with_ptcloud_.reset(new ApproximateSyncWithPtcloud(ApproximatePolicyWithPtcloud(4), sub_image_,
																			sub_disparity_, sub_rois_, sub_ptcloud_));
		approximate_sync_with_ptcloud_->registerCallback(
			boost::bind(&consistencyNode::realCallbackWithPtcloud, this, _1, _2, _3, _4));
	  }
	}

	// load the classifier configuration
	string dfn = nn + "/yml/constraints";  // default filename
	node_.param(nn + "/yaml_filename", con_.ymlFilename, dfn);
	con_.load();

	// The label of interest is:
	node_.param(ros::this_node::getName() + "/label", con_.label, 1);

	// mode to start with
	mode = "";
	node_.param(ros::this_node::getName() + "/mode", mode, string("none"));
	ROS_INFO("Selected mode: %s", mode.c_str());
	con_.mode = mode;

	// number of data to train with
	int dm = 1000;
	node_.param(ros::this_node::getName() + "/max_training_samples", con_.max_training_data, dm);

	if (mode.compare("detect") == 0)
	{
	  ROS_INFO("Starting in detection mode");
	}
	else if (mode.compare("train") == 0)
	{
	  ROS_ERROR("Starting in training mode is non-sensical");
	}
	else if (mode.compare("load") == 0)
	{  // mode is accumulate
	  ROS_INFO("Starting by loading consistency data");
	}
	else if (mode.compare("accumulate") == 0)
	{
	  ROS_INFO("Starting accumulation of training data");
	}
	else
	{
	  ROS_ERROR("INVALID STARTING MODE: switching to accumulate");
	  con_.mode = "accumulate";
	  node_.setParam(ros::this_node::getName() + "/mode", con_.mode);
	}
  }

  void imageCb2(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg)
  {
	// ROS_ERROR("Uso sam u imageCb2");
	RoisPtr P(new Rois);

	P->rois.clear();
	P->header.stamp = image_msg->header.stamp;
	P->header.frame_id = image_msg->header.frame_id;

	// cv::imshow("view", cv_bridge::toCvShare(disparity_msg, "bgr8")->image);
	// cv::waitKey(0);
	// ROS_ERROR("I sta cemo sad kad smo tu dosli?");
	RoiRect dR;
	dR.x = 0;
	dR.y = 0;
	dR.height = image_msg->height;
	dR.width = image_msg->width;
	dR.label = 0;
	P->rois.push_back(dR);
	// ROS_ERROR("Pozvat cu imageCb");
	imageCb(image_msg, disparity_msg, P);
  }

  void imageCb(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg,
			   const RoisConstPtr& rois_msg)
  {
	string nn = ros::this_node::getName();
	string mode = "";
	node_.param(nn + "/mode", mode, string("none"));
	con_.mode = mode;
	bool kinect_disparity_fix;
	node_.param(nn + "Kinect_Disparity_Fix", kinect_disparity_fix, false);
	float percent_done = 0.0;
	// Use CV Bridge to convert images
	// cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

	// ROS_ERROR("Could not convert from '%s' to 'bgr8'.", typeid(disparity_msg).name());
	
	cv_bridge::CvImagePtr cv_ptr;
	try
  	{
		// ROS_ERROR("PRE_TOCVCOPY");
    	cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
		// ROS_ERROR("POST_TOCVCOPY");
	  }
  	catch(cv_bridge::Exception& e)
  	{
    	ROS_ERROR("cv_bridge exception in bbox_info: %s", e.what());
    	return;
 	}
	
	
	// sensor_msgs::CvBridge bridge;
	// IplImage* ipl_im = bridge.imgMsgToCv(image_msg, "bgr8");

	assert(disparity_msg->image.encoding == sensor_msgs::image_encodings::TYPE_32FC1);

	// **********************************************************************//
	// between these comments lies a hack that accounts for the difference   //
	// between the focal lengths of the kinect's color camera and ir cameras //
	// TODO, account for this using proper calibration and projection        //
	cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width, (float*)&disparity_msg->image.data[0],
						 disparity_msg->image.step);

	if (kinect_disparity_fix)
	{
	  int nrows = 434;
	  int ncols = 579;
	  int row_offset = (dmat.rows - nrows) / 2;
	  int col_offset = (dmat.cols - ncols) / 2;
	  cv::Mat Scaled_Disparity(nrows, ncols, CV_32FC1);
	  resize(dmat, Scaled_Disparity, cv::Size(ncols, nrows), 0, 0, CV_INTER_NN);
	  for (int i = 0; i < dmat.rows; i++)
	  {
		float* rp = &dmat.at<float>(i, 0);
		for (int j = 0; j < dmat.cols; j++)
		{
		  *rp = 0.0;
		  rp++;
		}
	  }
	  for (int i = 0; i < nrows; i++)
	  {
		float* rp = &dmat.at<float>(i + row_offset, col_offset);
		float* rp2 = &Scaled_Disparity.at<float>(i, 0);
		for (int j = 0; j < ncols; j++)
		{
		  *rp = *rp2;
		  rp++;
		  rp2++;
		}
	  }
	}

	// **********************************************************************//

	// load ROS rois.msg data into vector of CvRect and labels for Classifier to use
	RoiRect roi;
	CvRect cvRoi;
	con_.cvRoisInput.clear();
	con_.labelsInput.clear();
	for (unsigned int i = 0; i < rois_msg->rois.size(); i++)
	{
	  roi = rois_msg->rois[i];
	  cvRoi.x = roi.x;
	  cvRoi.y = roi.y;
	  cvRoi.width = roi.width;
	  cvRoi.height = roi.height;
	  con_.cvRoisInput.push_back(cvRoi);
	  con_.labelsInput.push_back(roi.label);
	}

	// pass Color and Disparity Image to consistency object
	con_.color_image = cv_ptr->image;

	// con_.disparity_image=cv::Mat(ipl_imD);
	con_.disparity_image = dmat;

	// do the work of the callback
	if (con_.mode.compare("detect") == 0)
	{
	  con_.detect();
	}
	else if (con_.mode.compare("train") == 0)
	{
	  ROS_ERROR("starting training");
	  con_.train();
	  ROS_ERROR("done training switch to detect");
	  con_.mode = "detect";
	  node_.setParam(ros::this_node::getName() + "/mode", con_.mode);
	}
	else if (con_.mode.compare("accumulate") == 0)
	{
	  con_.accumulate();
	  percent_done = (float)con_.TD_.size() / (float)con_.max_training_data * 100.0;
	  if ((int)con_.TD_.size() >= con_.max_training_data)
	  {
		con_.mode = "train";
		node_.setParam(ros::this_node::getName() + "/mode", con_.mode);
	  }
	}
	else if (mode.compare("load") == 0)
	{
	  ROS_ERROR("consistency LOADING %s", con_.ymlFilename.c_str());
	  con_.load();
	  con_.mode = "detect";
	  node_.setParam(ros::this_node::getName() + "/mode", con_.mode);
	}
	else
	{
	  ROS_ERROR("INVALID MODE: switching to accumulate");
	  con_.mode = "accumulate";
	  node_.setParam(ros::this_node::getName() + "/mode", con_.mode);
	}

	// transfer results to output rois.msg
	output_rois_.rois.clear();
	output_rois_.header.stamp = image_msg->header.stamp;
	output_rois_.header.frame_id = image_msg->header.frame_id;

	for (unsigned int j = 0; j < con_.cvRoisOutput.size(); j++)
	{
	  roi.x = con_.cvRoisOutput[j].x;
	  roi.y = con_.cvRoisOutput[j].y;
	  roi.width = con_.cvRoisOutput[j].width;
	  roi.height = con_.cvRoisOutput[j].height;
	  roi.label = con_.labelsOutput[j];
	  output_rois_.rois.push_back(roi);
	}

	/*
		remove_overlap_Rois(output_rois_, non_overlapping_rois_);
		non_overlapping_rois_.header.stamp    = image_msg->header.stamp;
		non_overlapping_rois_.header.frame_id = image_msg->header.frame_id;
		int s1 = output_rois_.rois.size();
		int s2 = non_overlapping_rois_.rois.size();
		ROS_ERROR("output_rois_.size = %d non_overlapping size = %d",s1,s2);
	*/
	// publish results
	if (mode.compare("accumulate") == 0)
	{
	  ROS_INFO("ACCUMULATING TRAINING DATA: %5.1f%c done", percent_done, '%');
	}
	else
	{
	  ROS_INFO("PUBLISHED: %d Consistency ROIs", int(output_rois_.rois.size()));
	  pub_rois_.publish(output_rois_);
	  pub_Color_Image_.publish(image_msg);
	  pub_Disparity_Image_.publish(disparity_msg);
	}
	// ROS_ERROR("ZAVRSIO SAM OVO SVE TU.");
  }

  void realCallback(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg,
					const RoisConstPtr& rois_msg)
  {
	imageCb(image_msg, disparity_msg, rois_msg);
  }

  void realCallback2(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg)
  {
	imageCb2(image_msg, disparity_msg);
  }

  void realCallbackWithPtcloud(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg,
							   const RoisConstPtr& rois_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
	ROS_ERROR("PRE-WEEEE");
	imageCb(image_msg, disparity_msg, rois_msg);
	ROS_ERROR("WEEEE");
	pub_Ptcloud.publish(cloud_msg);
  }

  void realCallback2WithPtcloud(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg,
								const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
	ROS_ERROR("PRE-WEEEE2");
	imageCb2(image_msg, disparity_msg);
	ROS_ERROR("WEEEE2");
	pub_Ptcloud.publish(cloud_msg);
  }

  ~consistencyNode()
  {
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "consistency");
  ros::NodeHandle n;
  consistencyNode CN(n);
  ros::spin();
  return 0;
}
