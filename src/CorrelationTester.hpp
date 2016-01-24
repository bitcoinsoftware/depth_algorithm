#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gsl/gsl_math.h>
#include <exception>
#include <vector>
class CorrelationTester{
public:
  CorrelationTester();      
  ~CorrelationTester();        
  double getOpticalFrequencyCorrelation(const sensor_msgs::ImageConstPtr& msg);
  double getMomentumCorrelation(const sensor_msgs::ImageConstPtr& msg);
  void   getExtremalMatrixValues(const cv::Mat& mat);
  
  std::string OPENCV_OPTIC_WINDOW;
  std::string OPENCV_DEPTH_WINDOW;
  double maxDepthValue;
  double minDepthValue; 
  double depthRangeSectionWidth; //used to divide the depth range in sections of this lenght
  double minimalDepth; //used to filter out the image related to the infinity/undisclosed depth
  cv::Mat depth_image;

private:
  int member;
  
};
