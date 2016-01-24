#include "CorrelationTester.hpp"

CorrelationTester::CorrelationTester(){
	OPENCV_OPTIC_WINDOW = "Optic window";
	OPENCV_DEPTH_WINDOW = "Depth window";
	cv::namedWindow(OPENCV_OPTIC_WINDOW);
	cv::namedWindow(OPENCV_DEPTH_WINDOW);
	maxDepthValue = 0;
    minDepthValue = 0;
    depthRangeSectionWidth = 5.0;
    minimalDepth = 0.1;
	}
	
CorrelationTester::~CorrelationTester(){
	cv::destroyWindow(OPENCV_OPTIC_WINDOW);
	cv::destroyWindow(OPENCV_DEPTH_WINDOW);
	}

void CorrelationTester::getExtremalMatrixValues(const cv::Mat& mat){
	//cv::Point min_loc, max_loc;
	double tempMin, tempMax;
	//cv::minMaxLoc(mat, &minDepthValue, &maxDepthValue);//, &min_loc, &max_loc)
	cv::minMaxLoc(mat, &tempMin, &tempMax);//, &min_loc, &max_loc)
	if (tempMin < minDepthValue)
		minDepthValue = tempMin;
	if (tempMax > maxDepthValue)
		maxDepthValue = tempMax;
}

double CorrelationTester::getOpticalFrequencyCorrelation(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_mat_ptr;
	//gsl_stats_correlation (const double data1[], const size_t stride1, const double data2[], const size_t stride2, const size_t n)
    try{
		cv_mat_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		cv::Mat rgb_image;
		cv_mat_ptr->image.convertTo(rgb_image,CV_8U);
		
		if (minDepthValue != maxDepthValue){ //if the extremal values have been initialised with nonzero values
			int depthSectionAmount = static_cast<int>((maxDepthValue - minDepthValue)/depthRangeSectionWidth);
			double * depthSectionValueArray = new double[depthSectionAmount];
			double * opticalFrequencyArray  = new double[depthSectionAmount];
			
			cv::Mat depthRangeMask;
			cv::inRange(depth_image, cv::Scalar(0), cv::Scalar(minimalDepth), depthRangeMask);
			cv::Mat rgbImageForUndisclosedDepth; //it contains the rgb pixels related to the 0.0 depth
			rgb_image.copyTo(rgbImageForUndisclosedDepth, depthRangeMask);
			
			for (int i =0; i<depthSectionAmount; i++){ 
				*(depthSectionValueArray + i) = static_cast<double>(i);
				cv::Mat rgbMaskedImage;
				//we will use the depth image as a mask for the rgb image
				//we build a mask from the pixels of the depth image which value is in (i*depthRangeSectionWidth , (i+1)*depthRangeSectionWidth )
				cv::inRange(depth_image, cv::Scalar(i*depthRangeSectionWidth + minimalDepth), cv::Scalar((i+1)*depthRangeSectionWidth), depthRangeMask);
				rgb_image.copyTo(rgbMaskedImage, depthRangeMask);
				
				cv::Mat combine(rgb_image.size().height, rgb_image.size().width*3, CV_8UC1);
				cv::Mat left_roi(combine, cv::Rect(0, 0, rgb_image.size().width, rgb_image.size().height));
				rgb_image.copyTo(left_roi);
				cv::Mat center_roi(combine, cv::Rect(rgb_image.size().width, 0, rgb_image.size().width, rgb_image.size().height));
				rgbMaskedImage.copyTo(center_roi);
				cv::Mat right_roi(combine, cv::Rect(2*rgb_image.size().width, 0, rgb_image.size().width, rgb_image.size().height));
				rgbImageForUndisclosedDepth.copyTo(right_roi);

				cv::Size size(combine.size().width/2, combine.size().height/2);
				cv::resize(combine, combine, size);
				
				cv::imshow(OPENCV_DEPTH_WINDOW , combine);
				cv::waitKey(3);
			}
			
			delete depthSectionValueArray;
			delete opticalFrequencyArray;
		}
		
		/*
		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
		
		
		
		cv::Size image_size = cv_mat_ptr->image.size();
		
		
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
		*/
		
	  return 1;
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      }
    catch (cv::Exception& e){
	  ROS_ERROR("opencv exception: %s", e.what());
	  }
	return 0;
}

double CorrelationTester::getMomentumCorrelation(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_mat_ptr;
    try{
		//TODO image analysis - momentum - depth corellation
		cv_mat_ptr = cv_bridge::toCvCopy(msg);
		//cv::Mat depth_image;
		cv_mat_ptr->image.convertTo(depth_image,CV_8U, 25.5);
		//depthMat = depth_image;
		//cv::imshow(OPENCV_DEPTH_WINDOW , depth_image);
		//cv::waitKey(3);
		getExtremalMatrixValues(depth_image);
		
		
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch (cv::Exception& e){
	  ROS_ERROR("opencv exception: %s", e.what());
	}
	return 0;
}
