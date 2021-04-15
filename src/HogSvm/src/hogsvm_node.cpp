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

#include "HogSvm/hogsvm.hpp"

#include <sstream>
#include "ros/ros.h"

// Publish Messages
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"

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

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Includes for pointcloud
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace stereo_msgs;
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
using namespace cv;
using sensor_msgs::PointCloud2;

#define NUM_IMAGES 10

class HogSvmNode
{
private:
  // Define Node
  ros::NodeHandle node_;

  // Subscribe to Messages
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  message_filters::Subscriber<Image> sub_image_;
  message_filters::Subscriber<Rois> sub_rois_;
  message_filters::Subscriber<Image> sub_depth_;
  message_filters::Subscriber<PointCloud2> sub_ptcloud_;

  // Define the Synchronizer
  typedef ApproximateTime<Image, DisparityImage, Rois, Image> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  typedef ApproximateTime<Image, DisparityImage, Rois, PointCloud2, Image> ApproximatePolicyWithPtcloud;
  typedef message_filters::Synchronizer<ApproximatePolicyWithPtcloud> ApproximateSyncWithPtcloud;
  boost::shared_ptr<ApproximateSyncWithPtcloud> approximate_sync_with_ptcloud_;

  // Messages to Publish
  ros::Publisher pub_rois_;
  ros::Publisher pub_Color_Image_;
  ros::Publisher pub_Disparity_Image_;
  ros::Publisher pub_Ptcloud;

  Rois output_rois_;

  // Use visualization
  bool visualize_flag = false;

  enum
  {
	ACCUMULATE = 0,
	TRAIN,
	DETECT,
	EVALUATE,
	LOAD
  };

  // Define the Classifier Object
  HogSvm::HogSvmClassifier HSC_;
  int num_class1;
  int num_class0;
  int num_TP_class1;
  int num_FP_class1;
  int num_TP_class0;
  int num_FP_class0;


  // sgementation of background (to remove FP)
  cv::Mat initialDepthImg;
  int counterImages = 0;
  cv::Mat depthImagesArray[NUM_IMAGES];
  
  bool initialImageTaken;
  cv::Mat depth_img;


public:
  explicit HogSvmNode(const ros::NodeHandle& nh) : node_(nh)
  {
	initialImageTaken = false;
	initialDepthImg = cv::Mat::zeros(480, 640, CV_32FC1);
	// ROS_ERROR("Initial rows %d", initialDepthImg.rows);
	// ROS_ERROR("Initial cols %d", initialDepthImg.cols);

	num_class1 = 0;
	num_class0 = 0;
	num_TP_class1 = 0;
	num_FP_class1 = 0;
	num_TP_class0 = 0;
	num_FP_class0 = 0;

	string nn = ros::this_node::getName();
	int qs;
	if (!node_.getParam(nn + "/Q_Size", qs))
	{
	  qs = 3;
	}

	// get path and name of classifier and blocks
	string cfnm;  // classifier file name
	string hbnm;  // hog block file name
	node_.param(nn + "/classifier_file", cfnm, std::string("/test.xml"));
	node_.param(nn + "/Hog_Block_File", hbnm, std::string("/Blocks.xml"));
	HSC_.init(cfnm, hbnm);

	int NS;
	if (!node_.getParam(nn + "/num_Training_Samples", NS))
	{
	  NS = 350;
	  node_.setParam(nn + "/num_Training_Samples", NS);
	}
	HSC_.setMaxSamples(NS);

	// Published Messages
	pub_rois_ = node_.advertise<Rois>("HogSvmOutputRois", qs);
	pub_Color_Image_ = node_.advertise<Image>("HogSvmColorImage", qs);
	pub_Disparity_Image_ = node_.advertise<DisparityImage>("HogSvmDisparityImage", qs);

	// Subscribe to Messages
	sub_image_.subscribe(node_, "Color_Image", qs);
	sub_disparity_.subscribe(node_, "Disparity_Image", qs);
	sub_rois_.subscribe(node_, "input_rois", qs);
	sub_depth_.subscribe(node_, "Depth_Image", qs);

	// Check visualization param
	node_.param("/visualization/ptcloud", visualize_flag, false);
	if (visualize_flag)
	{
	  pub_Ptcloud = node_.advertise<PointCloud2>("HogSvmPtcloud", qs);
	  sub_ptcloud_.subscribe(node_, "PtCloud", qs);
	}

	// Sync the Synchronizer
	if (!visualize_flag)
	{
	  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(qs), sub_image_, sub_disparity_, sub_rois_, sub_depth_));
	  approximate_sync_->registerCallback(boost::bind(&HogSvmNode::realCallback, this, _1, _2, _3, _4));
	}
	else
	{
	  approximate_sync_with_ptcloud_.reset(new ApproximateSyncWithPtcloud(ApproximatePolicyWithPtcloud(qs), sub_image_,
																		  sub_disparity_, sub_rois_, sub_ptcloud_, sub_depth_));
	  approximate_sync_with_ptcloud_->registerCallback(
		  boost::bind(&HogSvmNode::realCallbackWithPtcloud, this, _1, _2, _3, _4, _5));
	}
  }

  int get_mode()
  {
	int callback_mode;
	std::string mode = "";
	node_.param(ros::this_node::getName() + "/mode", mode, std::string("none"));
	if (mode.compare("detect") == 0)
	{
	  callback_mode = DETECT;
	}
	else if (mode.compare("train") == 0)
	{
	  callback_mode = TRAIN;
	}
	else if (mode.compare("load") == 0)
	{
	  callback_mode = LOAD;
	}
	else if (mode.compare("evaluate") == 0)
	{
	  callback_mode = EVALUATE;
	}
	else if (mode.compare("accumulate") == 0)
	{
	  callback_mode = ACCUMULATE;
	}
	else  // default mode is accumulate
	{
	  callback_mode = ACCUMULATE;
	}
	return (callback_mode);
  }


  void imageCb(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg,
			   const RoisConstPtr& rois_msg, const ImageConstPtr& depth_msg)
  {
	bool label_all;
	vector<Rect> R_out;
	vector<int> L_out;
	string param_name;
	string cfnm;  // classifier file name
	string hbnm;  // hog block file name
	int numSamples;
	string nn = ros::this_node::getName();


	// TAKING THE INITAL DEPTH IMAGE
	if (!initialImageTaken)
	{
		if (counterImages >= NUM_IMAGES) // Counter high enough - go through depth images
		{
			std::vector<float> pixels;
			for (int i = 0; i < depthImagesArray[0].rows; i++)
			{
				for (int j = 0; j < depthImagesArray[0].cols; j++)
				{

					pixels.clear();
					for (int k = 0; k < NUM_IMAGES; k++)
					{

						if (!isnan(depthImagesArray[k].at<float>(i, j)))
						{
							pixels.push_back(depthImagesArray[k].at<float>(i, j));
						}
					}

					if (pixels.size() == 0) continue;
					else
					{

						sort(pixels.begin(), pixels.end());

						if(pixels.size() % 2 == 0)
						{

							initialDepthImg.at<float>(i, j) = (pixels[pixels.size()/2 - 1] + pixels[pixels.size()/2]) / 2;
						}
						else
						{
							initialDepthImg.at<float>(i, j) = pixels[pixels.size() / 2];
						}
					}

				}
			}

			initialImageTaken = true;
			ROS_INFO("Initial image saved.");
		}
		else // Counter low - save depth image
		{
			depthImagesArray[counterImages] = cv_bridge::toCvCopy(depth_msg, image_encodings::TYPE_32FC1)->image;
			counterImages++;
		}
	}

    depth_img = cv_bridge::toCvCopy(depth_msg,image_encodings::TYPE_32FC1)->image;

	// Use CV Bridge to convert images
	// sensor_msgs::CvBridge bridge;
	// IplImage* ipl_im = bridge.imgMsgToCv(image_msg, image_encodings::MONO8);
	// Mat im_gray = Mat(ipl_im);
	CvImagePtr cv_gray  = cv_bridge::toCvCopy(image_msg,image_encodings::MONO8);
	Mat im_gray  = cv_gray->image;

	// take the region of interest message and create vectors of ROIs and labels
	vector<int> L_in;
	vector<Rect> R_in;
	R_in.clear();
	L_in.clear();
	for (unsigned int i = 0; i < rois_msg->rois.size(); i++)
	{
	  int x = rois_msg->rois[i].x;
	  int y = rois_msg->rois[i].y;
	  int w = rois_msg->rois[i].width;
	  int h = rois_msg->rois[i].height;
	  int l = rois_msg->rois[i].label;
	

	  // Initial image is taken
	  // // check  
	  if(0)
      {
        int neighbourhood = 0;
        int rectCenterX = (int)(x + w/2);
        int rectCenterY = (int)(y + h/2);

		if(rectCenterY >= initialDepthImg.rows || rectCenterX >= initialDepthImg.cols || 
			rectCenterX + neighbourhood >= initialDepthImg.cols || rectCenterY + neighbourhood >= initialDepthImg.rows || 
			rectCenterX < 0 || rectCenterY < 0 || rectCenterX - neighbourhood < 0 || rectCenterY - neighbourhood < 0 || w <= 0 || h <= 0)
		{
			continue;
		}

		float mae;
		if (neighbourhood >= 2)
		{
			cv::Rect imgsPart(rectCenterX - neighbourhood, rectCenterY - neighbourhood, neighbourhood*2, neighbourhood*2);
			cv::Mat initImgPart = initialDepthImg(imgsPart);

			cv::Mat entryImgPart = depth_img(imgsPart);

			cv::Mat tempMAE;
			cv::absdiff(initImgPart, entryImgPart, tempMAE); // absolute difference between images
        	mae = (float)cv::sum(tempMAE)[0]/initImgPart.total(); // calculate every pixel
		}
		else
		{
			float depth_img_pixel = depth_img.at<float>(rectCenterX, rectCenterY);
			float init_depth_img_pixel = initialDepthImg.at<float>(rectCenterX, rectCenterY);

			mae = abs(depth_img_pixel - init_depth_img_pixel);
		}

		// ROS_ERROR("[DEBUG] Something5");
        // ROS_ERROR("[DEBUG] MAE = %.4f", mae);
        if(mae <= 1.0f) 
			continue;
      }

	  Rect R(x, y, w, h);
		/*
	  // REMOVE OVERLAPPING BOUNDING BOXES
	  
	  bool flag_overlap = false;
	  if(R_in.size() > 0)
	  {
		for (auto& it : R_in)
		{
			Rect temp = R & it;
			
			float percentageArea = (float)(temp.area()/(float)(R.area()+it.area()-temp.area()));
			ROS_ERROR("Area percentage = %f", percentageArea);
			
			if (percentageArea > 0.5f)
			{
				flag_overlap = true;
				break;
			}
		}

	  }
	  if (!flag_overlap)
	  {
		R_in.push_back(R);
		L_in.push_back(l);
	  }
	  */
	 	R_in.push_back(R);
		L_in.push_back(l);
	}





	// ROS_ERROR("[DEBUG] ----------");

	// get path and name of classifier and blocks
	param_name = nn + "/classifier_file";
	node_.param(param_name, cfnm, std::string("/test.xml"));
	param_name = nn + "/Hog_Block_File";
	node_.param(param_name, hbnm, std::string("/Blocks.xml"));
	param_name = nn + "/Save_Average_Roi";
	node_.param(param_name, HSC_.Save_Average_Roi_, false);
	param_name = nn + "/Ave_Roi_File";
	node_.param(param_name, HSC_.Ave_Roi_File_, std::string("/AveRoi.jpg"));

	// get parameter for controlling point along ROC
	param_name = nn + "/HogSvmThreshold";
	double temp = 0.0;
	node_.getParam(param_name, temp);
	HSC_.HogSvmThreshold_ = (float)temp;

	// do the work of the node
	switch (get_mode())
	{
	  case DETECT:
		label_all = false;
		HSC_.detect(R_in, L_in, im_gray, R_out, L_out, label_all);
		output_rois_.rois.clear();
		output_rois_.header.stamp = image_msg->header.stamp;
		output_rois_.header.frame_id = image_msg->header.frame_id;
		// ROS_INFO("HogSvm found %d objects", (int)L_out.size());
		for (unsigned int i = 0; i < R_out.size(); i++)
		{
		  RoiRect R;
		  R.x = R_out[i].x;
		  R.y = R_out[i].y;
		  R.width = R_out[i].width;
		  R.height = R_out[i].height;
		  R.label = L_out[i];
		  output_rois_.rois.push_back(R);
		}
		pub_rois_.publish(output_rois_);
		pub_Color_Image_.publish(image_msg);
		pub_Disparity_Image_.publish(disparity_msg);
		break;
	  case ACCUMULATE:
		if (!HSC_.initialized_)
		{
		  HSC_.init(cfnm, hbnm);
		}

		numSamples = HSC_.addToTraining(R_in, L_in, im_gray);
		param_name = nn + "/num_Training_Samples";
		int NS;
		if (node_.getParam(param_name, NS))
		{
		  ROS_ERROR("training(%d%c)", (int)(numSamples * 100.0 / NS), '%');
		  if (numSamples >= NS)
		  {
			float percent = (float)HSC_.numSamples_ * 100.0 / NS;
			ROS_INFO("ACCUMULATING: %6.1f%c", percent, '%');
			param_name = nn + "/mode";
			node_.setParam(param_name, std::string("train"));
			ROS_ERROR("DONE Accumulating, switching to train mode");
		  }
		}
		break;
	  case TRAIN:
		ROS_ERROR("TRAINING");
		// HSC_.train(cfnm);
		param_name = nn + "/mode";
		node_.setParam(param_name, std::string("evaluate"));
		ROS_ERROR("DONE TRAINING, switching to evaluate mode");
		break;
	  case EVALUATE:
	  {
		if (!HSC_.trained_)
		{
		  ROS_ERROR("HogSvm LOADING %s", cfnm.c_str());
		  ROS_ERROR("HogSvm LOADING %s", hbnm.c_str());
		  HSC_.init(cfnm, hbnm);
		}
		label_all = false;
		HSC_.detect(R_in, L_in, im_gray, R_out, L_out, label_all);

		int total0_in_list = 0;
		int total1_in_list = 0;
		int fp_in_list = 0;
		int tp_in_list = 0;

		// count the input labels
		for (unsigned int i = 0; i < R_in.size(); i++)
		{
		  if (L_in[i] == 0 || L_in[i] == -1)
			total0_in_list++;
		  if (L_in[i] == 1)
			total1_in_list++;
		}
		num_class0 += total0_in_list;
		num_class1 += total1_in_list;

		// count the output labels which have the correct and incorrect label
		for (unsigned int i = 0; i < R_out.size(); i++)
		{
		  if (L_out[i] == 1)
		  {
			tp_in_list++;
		  }
		  else
			fp_in_list++;
		}
		num_TP_class1 += tp_in_list;
		num_FP_class1 += fp_in_list;
		num_TP_class0 += total0_in_list - fp_in_list;
		num_FP_class0 += total1_in_list - tp_in_list;
		float tp1 = (float)num_TP_class1 / (float)num_class1 * 100.0;
		float tp0 = (float)num_TP_class0 / (float)num_class0 * 100.0;
		float fp1 = (float)num_FP_class1 / (float)num_class1 * 100.0;
		float fp0 = (float)num_FP_class0 / (float)num_class0 * 100.0;
		ROS_ERROR("TP0 = %6.2f FP0 =  %6.2f TP1 = %6.2f FP1 =  %6.2f", tp0, fp0, tp1, fp1);
	  }
	  break;
	  case LOAD:
		ROS_ERROR("HogSvm LOADING %s", cfnm.c_str());
		ROS_ERROR("HogSvm LOADING %s", hbnm.c_str());
		HSC_.init(cfnm, hbnm);
		param_name = nn + "/mode";
		node_.setParam(param_name, std::string("detect"));
		break;
	}  // end switch
  }

  void realCallback(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg,
					const RoisConstPtr& rois_msg, const ImageConstPtr& depth_msg)
  {
	imageCb(image_msg, disparity_msg, rois_msg, depth_msg);
  }

  void realCallbackWithPtcloud(const ImageConstPtr& image_msg, const DisparityImageConstPtr& disparity_msg,
							   const RoisConstPtr& rois_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const ImageConstPtr& depth_msg)
  {
	imageCb(image_msg, disparity_msg, rois_msg, depth_msg);
	pub_Ptcloud.publish(cloud_msg);
  }

  ~HogSvmNode()
  {
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HogSvm");
  ros::NodeHandle n;
  HogSvmNode HN(n);
  ros::spin();
  return 0;
}
