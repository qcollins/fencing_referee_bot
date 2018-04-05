#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  
#include <cmath>

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
    
    
    // Declare output image
    cv::Mat outImg = cv_ptr->image.clone();
    cv::Size s = cv_ptr->image.size();
    int pix_counter = 0;
        
    // Processing each frame
    for(int row = 0; row < s.height; row++) {
        for(int col = 0; col < s.width; col++) {
            cv::Vec3b color = (cv_ptr->image).at<cv::Vec3b>(cv::Point(col, row));
            int r = color[2];
            int g = color[1];
            int b = color[0];
            int green_thresh = g - 15;
            if(b > green_thresh || r > green_thresh) {
                color = cv::Vec3b(0,0,0);
            } else {
				pix_counter++;
			}
            
            outImg.at<cv::Vec3b>(cv::Point(col, row)) = color;
        }
    }
    //int limit = 
    if (pix_counter > 1000) {
		std::cout << "Hello!\n";
		// Should detect for 2s
	}

	//show input
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
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
