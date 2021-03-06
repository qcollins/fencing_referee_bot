#include <ros/ros.h>
#include <cmath>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  
#include <std_msgs/Bool.h>

static const std::string OPENCV_WINDOW 	= "Image window";
static const std::string OUT_WINDOW 	= "Red Filter";

static const int LIMIT = 800;

class ImageConverter
{
	ros::NodeHandle nh_;
	ros::Publisher	red_pub_; 

	image_transport::ImageTransport it_;
	image_transport::Subscriber 	image_sub_;
	image_transport::Publisher 		image_pub_;
	  
public:
  ImageConverter()
    : it_(nh_)
  {
		red_pub_ = nh_.advertise<std_msgs::Bool>("red", 1000);
		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, 
								   &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/filter_red/red_image", 1);

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
        //cv::inRange(cv_ptr->image, cv::Scalar(0, 0, 0), cv::Scalar(50, 50, 255), outImg);
		/* Processing each frame; only keeps pixel if it is red */
		for(int row = 0; row < s.height; row++) {
			for(int col = 0; col < s.width; col++) {
				cv::Vec3b color = (cv_ptr->image).at<cv::Vec3b>(cv::Point(col, row));
				int r = color[2];
				int g = color[1];
				int b = color[0];
				
				if(r >= 110 && g <= b && r > g) {
					pix_counter++;
				} else {
					color = cv::Vec3b(0,0,0);
				}
            
				outImg.at<cv::Vec3b>(cv::Point(col, row)) = color;
			}
		}
    
		/* Sends detection of red score light as a bool */
		/* TODO: Figure out limit/threshold */
		std_msgs::Bool is_red;
		is_red.data = false;
		if (pix_counter > LIMIT) {
			is_red.data = true;
		}
		
		red_pub_.publish(is_red);

		/* Any remaining output/functions */
		cv::imshow(OUT_WINDOW, outImg);
		cv::waitKey(3);
		image_pub_.publish(cv_ptr->toImageMsg());
	  }
};

int main(int argc, char** argv)
{
	  ros::init(argc, argv, "filter_red");
	  ImageConverter ic;
	  ros::spin();
	  return 0;
}
