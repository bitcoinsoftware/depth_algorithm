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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_2 = "Previous Image window";





class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat depth_image;
  cv::Mat edges_image;
  cv::Mat edges_image2;
  
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &ImageConverter::imageCb, this);
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
	// CONVERT IMAGE TO OPENCV TYPE
    cv_bridge::CvImagePtr cv_ptr;
    cv::RNG rng(12345);
    try{
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image.convertTo(depth_image,CV_8U, 1.0/255.0);
    
    
    
    
    int tunel_end_width = depth_image.size().width /5.0;
    int tunel_end_height= depth_image.size().height/5.0;
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = matToPoinXYZ(depth_image);
    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (cloud);
    //while (!viewer.wasStopped ())
    //  {
    //  }
    
    try{

		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
		double min, max;
		cv::Point min_loc, max_loc;
		cv::minMaxLoc(depth_image, &min, &max, &min_loc, &max_loc);
		
		cv::Size image_size = cv_ptr->image.size();
		
		
		double symetry_perspective_threshold = 3;
		int range_width = 2; // depth iteration step in the loop
		int d_max_observed   = int(max);
		double d_max = d_max_observed*2;
		double spatial_ratio = 4;
		double minimum_slope_value = 1/spatial_ratio;
		int range_amount = d_max_observed/range_width;

		int spatial_moment, weight_center_x, weight_center_y;
		std::vector<double> argumentVector, depthVectorXMin, depthVectorXMax, depthVectorYMin, depthVectorYMax;
		std::vector<double> centerXVector, centerYVector;
		// ITERATE THROUGH THE DEPTH IN range_width STEPS
		cv::Mat thresh_output;
		for (int i=0; i<= d_max_observed; i=i+range_width )
		{   //LEAVE ONLY IMAGE WITH DEPTH IN RANGE (i , i+range_width) 
			cv::threshold( depth_image, thresh_output, i, d_max_observed, cv::THRESH_TOZERO );
			cv::threshold( thresh_output, thresh_output, (i+range_width), d_max_observed, cv::THRESH_TOZERO_INV );
			//COUNT IMAGE MOMENTS AND SAVE THEM IN A VECTOR
			cv::Moments image_moment = cv::moments(thresh_output);
			spatial_moment = image_moment.m00;
			
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
				argumentVector.push_back(i);
				depthVectorXMin.push_back(min_x);
				depthVectorXMax.push_back(max_x);
				depthVectorYMin.push_back(min_y);
				depthVectorYMax.push_back(max_y);
			}
		}

		double x_min_slope = slope(argumentVector, depthVectorXMin);
		double x_max_slope = slope(argumentVector, depthVectorXMax);
		double y_min_slope = slope(argumentVector, depthVectorYMin);
		double y_max_slope = slope(argumentVector, depthVectorYMax);
		// IF THERE IS A SYMETRICAL SLOPE , IF PERSPECTIVE WAS DETECTED 
		if ((x_min_slope > 0 && x_max_slope < 0 && y_min_slope > 0 && y_max_slope < 0) && (std::abs(y_min_slope + y_max_slope) + std::abs(x_min_slope + x_max_slope)) < 2*symetry_perspective_threshold )// && std::abs(x_min_slope + x_max_slope) < 3)
		{
			// COUNT THE SLOPES OF THE PERSPECTIVE
			//double b_x_min = (x_center - tunel_end_width /2.0) - x_min_slope * d_max;
			//double b_x_max = (x_center + tunel_end_width /2.0) - x_max_slope * d_max;
			//double b_y_min = (y_center - tunel_end_height/2.0) - y_min_slope * d_max;
			//double b_y_max = (y_center + tunel_end_height/2.0) - y_max_slope * d_max;
			double x_center = center(centerXVector);
			//TODO dobor wielkosci konca korytarza
			double y_center = center(centerYVector);
			
			double b_x_min = x_center  - x_min_slope * d_max;
			double b_x_max = x_center  - x_max_slope * d_max;
			double b_y_min = y_center  - y_min_slope * d_max;
			double b_y_max = y_center  - y_max_slope * d_max;
			
			
			// COUNT THE CORRIDOR WIDTH 
			// DETECT VERTICAL LINES
			cv::Canny(depth_image, edges_image2, 1 , 2);
			int scale = 1; int delta = 0;
			cv::Sobel(edges_image2, edges_image2, CV_8U, 1, 0, 1, scale, delta);
			std::vector<cv::Vec2f> lines;
			cv::HoughLines(edges_image2, lines, 1, CV_PI/180, 70, 0, 0 );
			
			//TODO: IF PERSPECTIVE DETECTED COUNT THE TUNELL WIDTH BASED ON DISTANCE FROM VERTICAL LINES
			std::vector<double> lines_x_position_left, lines_x_position_right;
			std::vector<double> lines_depth_left, lines_depth_right;
			
			for( size_t i = 0; i < lines.size(); i++ )
			{
				float rho = lines[i][0];
				float theta = lines[i][1];
				
				if (theta>3.05){
					cv::Point pt1, pt2;
					double a = cos(theta);
					double b = sin(theta);
					double x0 = a*rho, y0 = b*rho;
					
					pt1.x = cvRound(x0 + 1000*(-b));
					pt1.y = cvRound(y0 + 1000*(a));
					pt2.x = cvRound(x0 - 1000*(-b));
					pt2.y = cvRound(y0 - 1000*(a));
					cv::line( edges_image2, pt1, pt2, cv::Scalar(70), 1, CV_AA);
					cv::Point circ_cen;
					circ_cen.x = cvRound(x0); circ_cen.y = y_center;
					cv::Scalar color = cv::Scalar(depth_image.at<uchar>(int(y_center), int(x0)));
					if (x0 < x_center){
						cv:circle(edges_image2, circ_cen, 2, color, 2);
						lines_x_position_left.push_back(x0); lines_depth_left.push_back(depth_image.at<uchar>(int(y_center), int(x0)));
					}
					else{
						circle(edges_image2, circ_cen, 2, color, 2);
						lines_x_position_right.push_back(x0); lines_depth_right.push_back(depth_image.at<uchar>(int(y_center), int(x0)));
					}	
				}
			}
			double lines_depth_slope_right = slope(lines_depth_right, lines_x_position_right); // dx/dd
			double av_x_right = accumulate(lines_x_position_right.begin(), lines_x_position_right.end(), 0) / static_cast<double>(lines_x_position_right.size());  //  
			double av_depth_right = accumulate(lines_depth_right.begin(), lines_depth_right.end(), 0) / static_cast<double>(lines_depth_right.size()); 
			double lines_depth_b_right = av_x_right - lines_depth_slope_right*av_depth_right;
			
			double lines_depth_slope_left  = slope(lines_depth_left, lines_x_position_left);
			double av_x_left = accumulate(lines_x_position_left.begin(), lines_x_position_left.end(), 0) / static_cast<double>(lines_x_position_left.size());
			double av_depth_left = accumulate(lines_depth_left.begin(), lines_depth_left.end(), 0) / static_cast<double>(lines_depth_left.size());
			double lines_depth_b_left = av_x_left - lines_depth_slope_left*av_depth_left;
			
			//std::cout<< "lines_depth_slope_right, "<<lines_depth_slope_right << " <<left slope right>>  " << lines_depth_slope_left<< " " << lines_depth_b_right <<std::endl;
			//std::printf("lines_depth_slope_right, %f , av_x_right , %f , av_depth_right , %f , lines_depth_b_right %f \n", lines_depth_slope_right, av_x_right, av_depth_right, lines_depth_b_right);
			//std::printf("lines_depth_slope_left, %f , av_x_left , %f , av_depth_left , %f , lines_depth_b_left %f \n"    , lines_depth_slope_left, av_x_left, av_depth_left, lines_depth_b_left);
			// DETECT HORIZONTAL LINES 
			cv::Canny(depth_image, edges_image, 1 , 2);
			cv::Sobel(edges_image, edges_image, CV_8U, 0, 1, 1, scale, delta);
			lines.clear();
			cv::HoughLines(edges_image, lines, 1, CV_PI/180, 70, 0, 0 );
			//TODO: IF PERSPECTIVE DETECTED COUNT THE TUNELL HEIGHT BASED ON DISTANCE FROM HORIZONTAL LINES
			for( size_t i = 0; i < lines.size(); i++ )
			{
				float rho = lines[i][0];
				float theta = lines[i][1];
				
				if (theta<1.6 && theta > 1.4){
					cv::Point pt1, pt2;
					double a = cos(theta);
					double b = sin(theta);
					double x0 = a*rho, y0 = b*rho;
					pt1.x = cvRound(x0 + 1000*(-b));
					pt1.y = cvRound(y0 + 1000*(a));
					pt2.x = cvRound(x0 - 1000*(-b));
					pt2.y = cvRound(y0 - 1000*(a));
					cv::line( edges_image2, pt1, pt2, cv::Scalar(170), 1, CV_AA);
					/*
					cv::Point circ_cen2;
					//TODO y should be an average
					// y = aa x +bb
					double aa = (pt1.y - pt2.y)/2000.0;
					double bb = pt1.y - aa * pt1.x ;
					std::cout <<"("<< pt1.x << " , " << pt1.y <<"), ( "<< pt2.x <<" , "<< pt2.y <<" )"<<aa << "  " <<bb << std::endl;
					
					circ_cen2.x = 100; circ_cen2.y = cvRound( 100*(aa) + bb);
					circle(edges_image2, circ_cen2, 2, cv::Scalar(255), 2); */
				}
			}
			
			
			double x_center_closest , y_center_closest, x_min_closest, x_max_closest;
			bool closest_center_set = false; 
			//COUNT x_min , x_max for the floor CHECK IF VALUES DONT EXCEED THE SIZE OF THE IMAGE
			//std::cout<<" NEW LOOP \n";
			//TODO  IMPROVE THE x_max_closest and x_min_closest
			//for (int d_i =range_max/2 ; d_i <= d_max; d_i++){
			
			
			for (int d_i =d_max_observed/2 ; d_i <= d_max; d_i++){
				//MIN VALUES
				int x_min = int(x_min_slope * d_i + b_x_min);
				//int x_min = int(lines_depth_slope_left * d_i + lines_depth_b_left);
				//int x_min = (int(x_min_slope * d_i + lines_depth_b_left) + int(x_min_slope * d_i + b_x_min))/2;
				//std::cout<< "(lines_depth_slope_left * d_i + lines_depth_b_left) "<< lines_depth_slope_left << " * "<< d_i << " + " << lines_depth_b_left << " = " << x_min <<std::endl;
				if (x_min<0 || x_min > image_size.width){continue;}
				
				//int x_min_p1 = int(x_min_slope * (d_i+1) + b_x_min);
				/*int x_min_p1 = int(lines_depth_slope_left * (d_i+1) + lines_depth_b_left);
				if (x_min_p1<0 || x_min_p1 > image_size.width){continue;}*/
				
				//int y_min = int(y_min_slope * d_i + b_y_min);
				int y_min = int(y_min_slope * d_i + b_y_min);
				if (y_min<0 || y_min > image_size.height) {continue;}
				
				//int y_min_p1 = int(y_min_slope * (d_i+1) + b_y_min);
				/*int y_min_p1 = int(y_min_slope * (d_i+1) + b_y_min);
				if (y_min_p1<0 || y_min_p1 > image_size.height) {continue;}*/
				
				//MAX VALUES
				int x_max = int(x_max_slope * d_i + b_x_max);
				//int x_max = int(lines_depth_slope_right * d_i + lines_depth_b_right);
				//int x_max = (int(x_max_slope * d_i + lines_depth_b_right)+int(x_max_slope * d_i + b_x_max))/2;
				if (x_max >= image_size.width || x_max < 0) {continue;}
				
				
				//int x_max_p1 = int(x_max_slope * (d_i+1) + b_x_max);
				/*int x_max_p1 = int(lines_depth_slope_right * (d_i+1) + lines_depth_b_right);
				if (x_max_p1 >= image_size.width || x_max_p1 < 0) {continue;}*/
				
				//int y_max = int(y_max_slope * d_i + b_y_max);
				int y_max = int(y_max_slope * d_i + b_y_max);
				if (y_max > image_size.height || y_max < 0) {continue;}
				
				//std::printf("d_i, x_min , x_max, y_max , y_min= %d , %d , %d, %d, %d\n", d_i, x_min, x_max, y_max, y_min);
				
				//int y_max_p1 = int(y_max_slope * (d_i+1) + b_y_max);
				/*int y_max_p1 = int(y_max_slope * (d_i+1) + b_y_max);
				if (y_max_p1 > image_size.height || y_max_p1 < 0) {continue;}*/
				
				if (closest_center_set == false){
					x_center_closest = (x_max+x_min)/2;
					x_min_closest = x_min;
					x_max_closest = x_max;
					y_center_closest = y_max;
					closest_center_set = true;
					//cv::Point pt;
					//pt.x = x_min_closest;
					//pt.y = y_center_closest;
					//cv::circle(depth_image, pt, , cv::Scalar(255));
					//cv::line( edges_image2, pt1, pt2, cv::Scalar(170), 1, CV_AA);
					}
			}
			
			std::printf( " y_center_closest, x_max_closest, x_min_closest = %f , %f, %f\n ", y_center_closest, x_max_closest, x_min_closest); 
		
			std::vector<double> depth_floor_vector;
			std::vector<double> y_floor_vector;
			double d_y = (y_center_closest - y_center);
			//COUNT  THE FLOOR SLOPE
			if (d_y !=0){
				double floor_line_slope     = (x_center_closest - x_center)/d_y;
				double floor_line_slope_min = ((x_min_closest+  av_x_left)/2    - x_center)/d_y;
				double floor_line_slope_max = ((x_max_closest +av_x_right)/2   - x_center)/d_y;

				double b = x_center - floor_line_slope * y_center;
				double b_min = x_center - floor_line_slope_min * y_center;
				double b_max = x_center - floor_line_slope_max * y_center;
				for (int y = image_size.height; y>= y_center; y--){
					int x = int(floor_line_slope * y + b);
					if(x >=0 && x < image_size.width && y >= 0 && y < image_size.height && depth_image.at<uchar>(int(y), int(x)) >0){
						depth_floor_vector.push_back(depth_image.at<uchar>(int(y), int(x)));
						y_floor_vector.push_back(y);
						}
					}
				double depth_floor_slope = slope(y_floor_vector, depth_floor_vector);
			
				//DRAW THE FLOOR
				double last_color=0;
				//for (int y = image_size.height; y >= y_center+tunel_end_height/2.0; y--){
				for (int y = (y_center_closest+ image_size.height)/2 ; y >= y_center+tunel_end_height/2.0; y--){
					int x = int(floor_line_slope * y + b);
					int x_min = int(floor_line_slope_min * y +b_min);
					int x_max = int(floor_line_slope_max * y +b_max);
					
					if(x >=0 && x < image_size.width && y >= 0 && y < image_size.height ){
						for (int i = x_min; i< x_max; i++){
							if (depth_image.at<uchar>( int(y), i )==0){
								depth_image.at<uchar>( int(y), i ) = last_color - depth_floor_slope;
								}
							}
						if (depth_image.at<uchar>( int(y), int(x) )!=0){
							last_color = depth_image.at<uchar>( int(y), int(x) );	
							}
						}
					}
				}
				
		
		}

		cv::imshow(OPENCV_WINDOW , depth_image);
		cv::imshow(OPENCV_WINDOW_2 , edges_image2);
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
