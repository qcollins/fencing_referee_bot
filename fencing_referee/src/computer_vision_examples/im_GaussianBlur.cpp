#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    //declare output image
    cv::Mat outImg;
	//cv::Mat original_image = cv_ptr->image;
	


	//Example : smoothing and image difference
	outImg = cv_ptr->image.clone(); 
	cv::GaussianBlur( outImg, outImg, cv::Size( 55, 55 ), 0, 0);
	
	for (unsigned int i = 0; i < outImg.rows; i ++){
		for (unsigned int j = 0; j < outImg.cols; j ++){
			//example of how to look up pixel values
			int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
			int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
			int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
			
			//example of how to look up pixel values
			int b_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[0];
			int g_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[1];
			int r_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[2];
			
			//outImg.at<cv::Vec3b>(i,j)[0] = b_ij;  
			//outImg.at<cv::Vec3b>(i,j)[1] = g_ij;  
			//outImg.at<cv::Vec3b>(i,j)[2] = r_ij; 
				
			
			outImg.at<cv::Vec3b>(i,j)[0] = b_ij2-b_ij;  
			outImg.at<cv::Vec3b>(i,j)[1] = g_ij2-g_ij;  
			outImg.at<cv::Vec3b>(i,j)[2] = r_ij2-r_ij;  
		
			
		}
	}
	
	

	//show input
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
	//show output
	cv::imshow(OUT_WINDOW, outImg);
    
	//pause for 3 ms
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
