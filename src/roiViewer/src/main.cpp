/*Software License Agreement (BSD License)
	
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

#include "ros/ros.h"//Used for launch file parameter parsing
#include <string>//Used for rois message vector
#include <vector>
#include <sstream>

//Included for files
#include <iostream>
#include <fstream>
#include "stdio.h"
#include "dirent.h"

//Publish Messages
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"

//Time Synchronizer
// NOTE: Time Synchronizer conflicts with QT includes may need to investigate
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//Subscribe Messages
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
// #include </home/valentin/cv_bridge/include/cv_bridge/CvBridge.h>

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
// #include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace sensor_msgs::image_encodings;
using namespace roi_msgs;
using namespace cv;

//NOTE: Where is the best place to put these
//typedef ApproximateTime<Image, DisparityImage> ApproximatePolicy;
//typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

class roiViewerNode
{
	private:
	// Define Node
	ros::NodeHandle node_;
	
	// Subscribe to Messages
	message_filters::Subscriber<Image> sub_image_;
	message_filters::Subscriber<Rois> sub_rois_;
	
	// Define the Synchronizer
	typedef ApproximateTime<Image, Rois> ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ApproximateSync> approximate_sync_;
	
	// Launch file Parameters
	int label;
	
	public:
	
	explicit roiViewerNode(const ros::NodeHandle& nh): node_(nh)
	{
		label = 0;
		
		//Read mode from launch file
		std::string mode="";
		node_.param(ros::this_node::getName() + "/mode", mode, std::string("none"));
		ROS_INFO("Selected mode: %s",mode.c_str());
		
		if(mode.compare("roi_display") == 0)
		{
			ROS_INFO("MODE: %s", mode.c_str());
			
			//Get the image width and height                        
			node_.param(ros::this_node::getName() + "/label", label, 0);
			
			// Subscribe to Messages
			sub_image_.subscribe(node_, "input_image", 20);
			sub_rois_.subscribe(node_, "input_rois", 20);
			
			// Sync the Synchronizer
			approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(20), sub_image_, sub_rois_));
			
			approximate_sync_->registerCallback(boost::bind(&roiViewerNode::imageCb, this, _1, _2));
		}
		else
		{
			ROS_INFO("Unknown mode:%s  Please set to {roi_display} in roiViewer.launch",mode.c_str());
		}
		
		// Visualization
		// cv::namedWindow("Detections", 0 ); // non-autosized
		// cv::startWindowThread();
		
	}
	
	void imageCb(const ImageConstPtr& image_msg, const RoisConstPtr& rois_msg){
		
		std::string filename = image_msg->header.frame_id.c_str();
		std::string imgName = filename.substr(filename.find_last_of("/")+1);
		
		ROS_ERROR("roiViewer Callback called for image: %s", imgName.c_str());
		
		//Use CV Bridge to convert images       
		
		// ROS_ERROR("%s aaaaaaaa", image_msg->encoding);
		cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
		// ROS_ERROR("%s", image_msg->encoding);
		// sensor_msgs::CvBridge bridge;
		// IplImage* ipl_im = bridge.imgMsgToCv(image_msg, "bgr8");
		// cv::Mat cv_color = cv::Mat(ipl_im); 
		
		RoiRect roi;
		
		//For each roi in rois message
		ROS_ERROR("ROIS size: %d",rois_msg->rois.size());
		for(unsigned int i=0;i<rois_msg->rois.size();i++)
		{
			roi = rois_msg->rois[i];
			//Check to see if the label is to be displayed
			if(roi.label==label){
				//Add the roi label to the image
				//ROS_INFO("ROI found for image");
				Point ptUpperLeft = Point(roi.x,roi.y);
				Point ptLowerRight = Point(roi.x+roi.width,roi.y+roi.height);
				
				rectangle(cv_color->image,ptUpperLeft,ptLowerRight,Scalar(255,255,255));
			}	
		}
		
		// Display the cv image
		// cv::namedWindow("ROI Color Image",1);
		cv::imshow("Detections", cv_color->image);
		cv::waitKey(1);
	}
	
	~roiViewerNode(){}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roiViewer");
	ros::NodeHandle n;
	roiViewerNode roiViewerNode(n);
	ros::spin();
	
	return 0;
}

