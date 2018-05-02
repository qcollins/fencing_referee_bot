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
#include <filter.h>

using namespace cv;
using namespace cv_bridge;

static const std::string OPENCV_WINDOW 	= "Image window";
static const std::string OUT_WINDOW 	= "Green Filter";

int pix_counter = 0;

Vec3b pix_fxn(Vec3b color, int col) {
    int r = color[2];
    int g = color[1];
    int b = color[0];
    int green_thresh = g - 15;
    if(b <= green_thresh && r <= green_thresh) {
        pix_counter++;
    } else {
        color = Vec3b(0,0,0);
    }
    return color;
}

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
		image_pub_ = it_.advertise("/filter_green/green_image", 1);

		namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
		destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
		CvImagePtr cv_ptr = generate_cv_ptr(msg);
		
		/* Variables Initialization (2) */
		Mat outImg = process_image(cv_ptr, pix_fxn);
	    
		/* Sends detection of green score light as a bool */
		/* TODO: Figure out limit/threshold */
		std_msgs::Bool is_green;
		is_green.data = false;
		if (pix_counter > LIMIT) {
			is_green.data = true;
		}
		
		green_pub_.publish(is_green);

		/* Any remaining output/functions */
		imshow(OUT_WINDOW, outImg);
		waitKey(3);
		image_pub_.publish(cv_ptr->toImageMsg());
	  }
};

int main(int argc, char** argv)
{
	  ros::init(argc, argv, "filter_green");
	  ImageConverter ic;
	  ros::spin();
	  return 0;
}
