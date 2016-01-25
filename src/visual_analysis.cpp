#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "CorrelationTester.hpp"

//static const std::string OPENCV_WINDOW_2 = "Previous Image window";
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//typedef CorrelationTester::CorrelationTester correlationTest;

class VisualAnalysis{
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &VisualAnalysis::pclCb, this);
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber rgb_sub_;
  image_transport::Subscriber depth_sub_;
  
  sensor_msgs::ImageConstPtr 		rgbMsgPtr;
  sensor_msgs::ImageConstPtr 		depthMsgPtr;
  sensor_msgs::PointCloud2ConstPtr 	pclMsgPtr;
  
  CorrelationTester* corrTestPtr;
  double momentumCorrelation;
  double opticalFrequencyCorrelation;
  
public:
	VisualAnalysis();
	~VisualAnalysis();

  void depthCb(const sensor_msgs::ImageConstPtr& msg){
	depthMsgPtr = msg;
	std::cout << corrTestPtr->getDepthData(msg) << "\n";
  }

  void rgbCb(const sensor_msgs::ImageConstPtr& msg){
	rgbMsgPtr = msg;
	std::cout << corrTestPtr->getOpticalFrequencyCorrelation(msg) << "\n";
  }
  
  void pclCb(const sensor_msgs::PointCloud2ConstPtr& msg){
	pclMsgPtr = msg;
	}
};

VisualAnalysis::VisualAnalysis() : it_(nh_){
    rgb_sub_   = it_.subscribe("/camera/rgb/image_rect_color", 1, &VisualAnalysis::rgbCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image_rect", 1, &VisualAnalysis::depthCb, this);
    corrTestPtr = new CorrelationTester;
    //cv::namedWindow(OPENCV_WINDOW);
    //cv::namedWindow(OPENCV_WINDOW_2);
	}

VisualAnalysis::~VisualAnalysis() {
    //cv::destroyWindow(OPENCV_WINDOW);
    //cv::destroyWindow(OPENCV_WINDOW_2);
	}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualAnalysis");
  VisualAnalysis ic;
  printf ("Analysis started \n");
  ros::spin();
  return 0;
}

