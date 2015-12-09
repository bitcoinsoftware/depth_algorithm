#include <ros/ros.h>
#include <string>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_2 = "Previous Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat previous_image;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW_2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW_2);
  }


void type2str(int type) {
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) 
  {
    case CV_8U:  ROS_INFO("8U");   break;
    case CV_8S:  ROS_INFO("8S");   break;
    case CV_16U: ROS_INFO("16U");  break;
    case CV_16S: ROS_INFO("16S");  break;
    case CV_32S: ROS_INFO("32S");  break;
    case CV_32F: ROS_INFO("32F");  break;
    case CV_64F: ROS_INFO("64F");  break;
    default:     ROS_INFO("User"); break;
  }
}

double slope(const std::vector<double>& x, const std::vector<double>& y) {
    const int n    = x.size();
    const double s_x  = std::accumulate(x.begin(), x.end(), 0.0);
    const double s_y  = std::accumulate(y.begin(), y.end(), 0.0);
    const double s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
    const double s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
    const double a    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
    return a;
}

double center(const std::vector<double>& x)
{
	const double s_x = std::accumulate(x.begin(), x.end(), 0.0);
	return s_x/x.size();
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::RNG rng(12345);
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image.convertTo(previous_image,CV_8U, 1.0/255.0);
    cv::Mat thresh_output;
    try{
		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
		
		int range_width = 2; 
		int range_max   = 39; //TODO generate in a dynamic way
		double spatial_ratio =8;
		double minimum_slope_value = 1/spatial_ratio;
		int range_amount = range_max/range_width;
		
		int spatial_moment, weight_center_x , weight_center_y;
		std::vector<double> argumentVector, depthVectorXMin, depthVectorXMax, depthVectorYMin, depthVectorYMax;
		std::vector<double> centerXVector , centerYVector;
		
		for (int i=0 ; i<= range_max; i=i+range_width )
		{
			cv::threshold( previous_image, thresh_output, i, range_max, cv::THRESH_TOZERO );
			cv::threshold( thresh_output, thresh_output, (i+1), range_max, cv::THRESH_TOZERO_INV );
			
			std::ostringstream _from , _to;
			_from << range_width*i;
			_to   << range_width*(i+1);
			//std::string window_title( "Range from " + _from.str() + " to " + _to.str() );
			cv::Moments image_moment = cv::moments(thresh_output);
			
			spatial_moment = image_moment.m00;
			
			//std::cout << "D= " << i <<" Spatial moment " << spatial_moment << std::endl;
			
			if (spatial_moment > 1){
				weight_center_x = image_moment.m10 / spatial_moment;
				weight_center_y = image_moment.m01 / spatial_moment;
				
				centerXVector.push_back(weight_center_x);
				centerYVector.push_back(weight_center_y);
								
				double half_spatial_moment_sqrt = sqrt(spatial_moment)/spatial_ratio;
				double min_x = weight_center_x - half_spatial_moment_sqrt;
				double max_x = weight_center_x + half_spatial_moment_sqrt;
				
				double min_y = weight_center_y - half_spatial_moment_sqrt;
				double max_y = weight_center_x + half_spatial_moment_sqrt;
				
				//std::cout <<" min_x= "<<min_x<<" max_x= "<<max_x<<" min_y= "<<min_y<<" max_y= "<<max_y<<std::endl;
				
				argumentVector.push_back(i);
				depthVectorXMin.push_back(min_x);
				depthVectorXMax.push_back(max_x);
				depthVectorYMin.push_back(min_y);
				depthVectorYMax.push_back(max_y);
			}
		}
		
		double d_max = 2 * range_max;
		
		double x_center = center(centerXVector);
		double y_center = center(centerYVector);

		double x_min_slope = slope(argumentVector, depthVectorXMin);
		double x_max_slope = slope(argumentVector, depthVectorXMax);
		double y_min_slope = slope(argumentVector, depthVectorYMin);
		double y_max_slope = slope(argumentVector, depthVectorYMax);
		

		//if (x_min_slope > minimum_slope_value && x_max_slope < minimum_slope_value && y_min_slope > minimum_slope_value  && y_max_slope < -minimum_slope_value )
		if (x_min_slope > 0 && x_max_slope < 0 && y_min_slope > 0 && y_max_slope < 0 )
		{
			std::cout << "Perspective detected X min \n";
			double b_x_min = x_center - x_min_slope * d_max;
			double b_x_max = x_center - x_max_slope * d_max;
			double b_y_min = y_center - y_min_slope * d_max;
			double b_y_max = y_center - y_max_slope * d_max;
			std::cout << "x_min = " << x_min_slope << "d + " << b_x_min << "  X_max = " << x_max_slope << "d + " << b_x_max <<std::endl;
			std::cout << "x_min = " << y_min_slope << "d + " << b_x_min << "  Y_max = " << y_max_slope << "d + " << b_x_max <<std::endl;
			//GENERATE DEPTH IMAGE
			//generate the deepest square
			for (int d_i =range_max ; d_i < d_max; d_i++){
				int x_min = int(x_min_slope * d_i + b_x_min);
				int x_max = int(x_max_slope * d_i + b_x_max);
				int y_min = int(y_min_slope * d_i + b_y_min);
				int y_max = int(y_max_slope * d_i + b_y_max);
				
				int x_min_p1 = int(x_min_slope * (d_i+1) + b_x_min);
				int x_max_p1 = int(x_max_slope * (d_i+1) + b_x_max);
				int y_min_p1 = int(y_min_slope * (d_i+1) + b_y_min);
				int y_max_p1 = int(y_max_slope * (d_i+1) + b_y_max);
				
				for (int x= x_min; x<= x_max; x++){
					for(int y = y_min; x<= y_max; y++){
						TODO
						if (x < x_min_p1 && y < y_min_p1 ){ 
							if (x > x_max_p1 && y> y_max_p1){
								if (previous_image.at<float>(y, x)==0) previous_image.at<float>(y, x) = d_i;
							}
						} else{
							
						}
					}
				}
				
				//cv::Point point_center((x_min+x_max)/2, (y_min+y_max)/2);
				//int radius = ((x_max-x_min)+(y_max-y_min))/2;
				/*cv::Point point_l_up(x_min, y_min);
				cv::Point point_r_up(x_max, y_min);
				cv::Point point_l_down(x_min, y_max);
				cv::Point point_r_down(x_max, y_max);*/
				//cv::rectangle(previous_image, point_l_up, point_r_down, cv::Scalar(d_i), 10);
				//cv::circle(Mat& img, Point center, int radius, const Scalar& color);
				//cv::circle(previous_image, point_center, radius,cv::Scalar(d_i));
			}
			//std::cout << "(x_min , y_min) = ( " << x_min << " , " << y_min << " ) " << "(x_max , y_max) = ( " << x_max << " , " << y_max << " ) \n";
			//void rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
			
		}
		cv::imshow(OPENCV_WINDOW , previous_image);
	}
	catch (cv::Exception& e){
		ROS_ERROR("cv exception: %s", e.what());
	} 
	
	double minVal; 
	double maxVal; 
    
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
