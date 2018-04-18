#include <ros/ros.h>
#include <cmath>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  
#include <std_msgs/Bool.h>

static const std::string OPENCV_WINDOW 	= "Image window";
static const std::string OUT_WINDOW 	= "Green Filter";

static const int LIMIT = 1000;

class ImageConverter
{
	ros::NodeHandle nh_;
	ros::Publisher	green_pub_; 

	image_transport::ImageTransport it_;
	image_transport::Subscriber 	image_sub_;
	image_transport::Publisher 		image_pub_;
	  
public:
  ImageConverter()
    : it_(nh_)
  {
		green_pub_ = nh_.advertise<std_msgs::Bool>("green", 1000);
		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, 
								   &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/green_filter/green_image", 1);

		cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
		cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
		/* Variable Initialization (1) */
		cv_bridge::CvImagePtr cv_ptr;
    
		/* Exceptions */
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		/* Variables Initialization (2) */
		cv::Mat outImg = cv_ptr->image.clone();
		cv::Size s = cv_ptr->image.size();
		int pix_counter = 0;
            
		/* Processing each frame; only keeps pixel if it is green */
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
    
		/* Sends detection of green score light as a bool */
		/* TODO: Figure out limit/threshold */
		std_msgs::Bool is_green;
		is_green.data = false;
		if (pix_counter > LIMIT) {
			is_green.data = true;
		}
		
		//ROS_INFO("%d", is_green.data);
		green_pub_.publish(is_green);

		/* Any remaining output/functions */
		cv::imshow(OUT_WINDOW, outImg);
		cv::waitKey(3);
		image_pub_.publish(cv_ptr->toImageMsg());
	  }
};

int main(int argc, char** argv)
{
	  ros::init(argc, argv, "green_filter");
	  ImageConverter ic;
	  ros::spin();
	  return 0;
}
