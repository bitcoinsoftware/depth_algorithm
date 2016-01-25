#include "CorrelationTester.hpp"

CorrelationTester::CorrelationTester(){
	//OPENCV_OPTIC_WINDOW = "Optic window";
	OPENCV_DEPTH_WINDOW = "Depth window";
	//cv::namedWindow(OPENCV_OPTIC_WINDOW);
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

double CorrelationTester::getCorrelationCoefficient(int arrayLength, double array1[], double array2[]){
	//TODO
	return -1;
}

double CorrelationTester::getOpticalFrequencyCorrelation(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_mat_ptr;
    try{
		cv_mat_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		cv::Mat rgb_image;
		cv_mat_ptr->image.convertTo(rgb_image,CV_8U);
		
		if (minDepthValue != maxDepthValue){ //if the extremal values have been initialised with nonzero values
			int depthSectionAmount = static_cast<int>((maxDepthValue - minDepthValue)/depthRangeSectionWidth);
			double * depthSectionValueArray = new double[depthSectionAmount];
			double * opticalFrequencyArray  = new double[depthSectionAmount];
			double * spacialMomentumArray   = new double[depthSectionAmount];
			
			cv::Mat depthRangeMask;
			cv::inRange(depth_image, cv::Scalar(0), cv::Scalar(minimalDepth), depthRangeMask);
			cv::Mat rgbImageForUndisclosedDepth; //it contains the rgb pixels related to the 0.0 depth
			rgb_image.copyTo(rgbImageForUndisclosedDepth, depthRangeMask);
			
			for (int i =0; i<depthSectionAmount; i++){ 
				*(depthSectionValueArray + i) = static_cast<double>(i); //save the depth value
				cv::Mat rgbMaskedImage;
				//we will use the depth image as a mask for the rgb image
				//we build a mask from the pixels of the depth image which value is in (i*depthRangeSectionWidth , (i+1)*depthRangeSectionWidth )
				cv::inRange(depth_image, cv::Scalar(i*depthRangeSectionWidth + minimalDepth), cv::Scalar((i+1)*depthRangeSectionWidth), depthRangeMask);
				rgb_image.copyTo(rgbMaskedImage, depthRangeMask);
				*(spacialMomentumArray + i) = static_cast<double>(cv::moments(rgb_image).m00); //register the spacial momentum of the image corespoding to the depth = i
				cv::Mat dft;//we count the momentum of the images dft, because momentum(dft) ~ optical_frequency
				getImageDFT(rgbMaskedImage, dft);
				*(opticalFrequencyArray + i) = static_cast<double>(cv::moments(dft).m00);
				//glue 3 matrices together so we can display them in one cv window
				cv::Mat combine(rgb_image.size().height, rgb_image.size().width*3, CV_8UC1);
				cv::Mat left_roi(combine, cv::Rect(0, 0, rgb_image.size().width, rgb_image.size().height));
				rgb_image.copyTo(left_roi);
				cv::Mat center_roi(combine, cv::Rect(rgb_image.size().width, 0, rgb_image.size().width, rgb_image.size().height));
				rgbMaskedImage.copyTo(center_roi);
				cv::Mat right_roi(combine, cv::Rect(2*rgb_image.size().width, 0, rgb_image.size().width, rgb_image.size().height));
				rgbImageForUndisclosedDepth.copyTo(right_roi);
				cv::imshow(OPENCV_DEPTH_WINDOW , combine);
				cv::waitKey(3);
			}
			//gsl_stats_correlation (const double data1[], const size_t stride1, const double data2[], const size_t stride2, const size_t n)
			//count the correlation
			//double opticalFrequencyCorrelation = gsl::gsl_stats_correlation(opticalFrequencyArray, 1, depthSectionValueArray, 1, depthSectionAmount);
			double opticalFrequencyCorrelation = getCorrelationCoefficient(depthSectionAmount, opticalFrequencyArray, depthSectionValueArray);
			//double corr = Stats::Moments::correlation<int>(4, x , y);
			std::cout<< opticalFrequencyCorrelation << "\n";
			delete depthSectionValueArray;
			delete opticalFrequencyArray;
			delete spacialMomentumArray;
			
		}
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

double CorrelationTester::getDepthData(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_mat_ptr;
    try{
		cv_mat_ptr = cv_bridge::toCvCopy(msg);
		cv_mat_ptr->image.convertTo(depth_image,CV_8U, 25.5);
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

void CorrelationTester::getImageDFT(const cv::Mat& mat, cv::Mat& magI){
	cv::Mat padded; //expand input image to optimal size
    int m = cv::getOptimalDFTSize( mat.rows );
    int n = cv::getOptimalDFTSize( mat.cols ); // on the border add zero values
    cv::copyMakeBorder(mat, padded, 0, m - mat.rows, 0, n - mat.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
    cv::dft(complexI, complexI);            // this way the result may fit in the source matrix
    
    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    cv::split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    cv::magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    magI = planes[0];

    magI += cv::Scalar::all(1);                    // switch to logarithmic scale
    cv::log(magI, magI);
    

	
    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(cv::Rect(0, 0, magI.cols & -2, magI.rows & -2));
    

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    cv::Mat q0(magI, cv::Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    cv::Mat q1(magI, cv::Rect(cx, 0, cx, cy));  // Top-Right
    cv::Mat q2(magI, cv::Rect(0, cy, cx, cy));  // Bottom-Left
    cv::Mat q3(magI, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

    cv::Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    cv::normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
                                                // viewable image form (float between values 0 and 1).

    cv::resize(magI, magI, mat.size());
    
    //cv::imshow("magI" , magI);
	//cv::waitKey(3);

	}
	

