#include <ros/ros.h>
#include <string>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <vector>
#include <math.h>
#include <exception>
#include <algorithm>
#include <iostream>
#include <numeric>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

static const std::string OPENCV_WINDOW = "RGB window";
static const std::string OPENCV_WINDOW_2 = "Previous Image window";
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class VisualAnalysis{
  ros::NodeHandle nh_;
  ros::Publisher pub = nh_.advertise<PointCloud> ("points2", 1);
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_sub_;
  cv::Mat depth;
  
  
public:
  VisualAnalysis() : it_(nh_) {
    depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &VisualAnalysis::analysis, this);
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW_2);
	}

  ~VisualAnalysis() {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW_2);
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage, Eigen::Matrix3f& rgbIntrinsicMatrix) {
	PointCloud::Ptr createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage, Eigen::Matrix3f& rgbIntrinsicMatrix) {
		float rgbFocalInvertedX = 1/rgbIntrinsicMatrix(0,0);	// 1/fx
		float rgbFocalInvertedY = 1/rgbIntrinsicMatrix(1,1);	// 1/fy

		PointCloud::Ptr outputPointcloud (new PointCloud);
		outputPointcloud->width = depthImage.cols;
		outputPointcloud->height = depthImage.rows;
		
		pcl::PointXYZ newPoint;
		
		for (int i=0;i<depthImage.rows;i++){
			for (int j=0;j<depthImage.cols;j++){
				float depthValue = depthImage.at<float>(i,j);
				
				if (depthValue != 0){
					// Find 3D position respect to rgb frame:
					newPoint.z = depthValue;
					newPoint.x = (j - rgbIntrinsicMatrix(0,2)) * newPoint.z * rgbFocalInvertedX;
					newPoint.y = (i - rgbIntrinsicMatrix(1,2)) * newPoint.z * rgbFocalInvertedY;
					outputPointcloud->points.push_back(newPoint);
				}
				else{	
					newPoint.z = std::numeric_limits<float>::quiet_NaN();
					newPoint.x = std::numeric_limits<float>::quiet_NaN();
					newPoint.y = std::numeric_limits<float>::quiet_NaN();

					outputPointcloud->points.push_back(newPoint);
				}
			}
		} 
	return outputPointcloud;
	}

  
  void analysis(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_depth_ptr;
    cv::RNG rng(12345);
    //printf ("Started analysis function\n");
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	try
	{
		/*
		# Intrinsic camera matrix for the raw (distorted) images.
		#     [fx  0 cx]
		# K = [ 0 fy cy]
		#     [ 0  0  1] */
		float fx = 570.34222412f;
		float fy = 570.34222412f;
		float a = -0.0030711f;
		float b = 3.3309495f;
		float cx = 319.5f;
		float cy = 239.5f;
		Eigen::Matrix3f rgbIntrinsicMatrix(3,3);
		/*
		rgbIntrinsicMatrix(0,0) = fx;
		rgbIntrinsicMatrix(0,1) = 0;
		rgbIntrinsicMatrix(0,2) = cx;
		
		rgbIntrinsicMatrix(1,0) = 0;
		rgbIntrinsicMatrix(1,1) = fy;
		rgbIntrinsicMatrix(1,2) = cy;	
		
		rgbIntrinsicMatrix(2,0) = 0;
		rgbIntrinsicMatrix(2,1) = 0;
		rgbIntrinsicMatrix(2,2) = 1;*/
		rgbIntrinsicMatrix(0,0) = fx;
		rgbIntrinsicMatrix(1,0) = 0;
		rgbIntrinsicMatrix(2,0) = cx;
		
		rgbIntrinsicMatrix(0,1) = 0;
		rgbIntrinsicMatrix(1,1) = fy;
		rgbIntrinsicMatrix(2,1) = cy;	
		
		rgbIntrinsicMatrix(0,2) = 0;
		rgbIntrinsicMatrix(1,2) = 0;
		rgbIntrinsicMatrix(2,2) = 1;
		
		cv_depth_ptr->image.convertTo(depth, CV_8U, 1.0/255.0);
		
		PointCloud::Ptr cloud = createPointcloudFromRegisteredDepthImage(depth, rgbIntrinsicMatrix);
		// Create the filtering object
		
		/*PointCloud::Ptr cloud_filtered  (new PointCloud);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		
		sor.setInputCloud (cloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		
		sor.filter (*cloud_filtered);
		
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud);
		sor.setLeafSize (0.1f, 0.1f, 0.1f);
		sor.filter (*cloud_filtered);
		
		printf ("Cloud_filtered: width = %d, height = %d , size = %d\n", cloud_filtered->width, cloud_filtered->height, int(cloud_filtered->points.size()));
		
		*/
		
		//cloud_filtered->header.frame_id = "some_tf_frame";
		
		// We now want to create a range image from the above point cloud, with a 1deg angular resolution
		float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
		float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
		float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
		Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		
		float noiseLevel=0.00;
		float minRange = 0.0f;
		int borderSize = 1;
  
		pcl::RangeImage rangeImage;
		
		sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg(*cloud, cloud2);
        cloud2.header.frame_id = "map";
        cloud2.header.stamp = ros::Time::now();
        pub.publish(cloud2);
        printf ("Cloud: width = %d, height = %d , size = %d\n", cloud->width, cloud->height, int(cloud->points.size()));

		//rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
		
		//pcl::io::savePCDFile("test_pcd.pcd", cloud2);
		
		
		
	}
	catch (cv::Exception& e){
		ROS_ERROR("cv exception: %s", e.what());
	}
	cv::waitKey(3);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualAnalysis");
  VisualAnalysis ic;
  printf ("Visual analysis started \n");
  ros::spin();
  return 0;
}

