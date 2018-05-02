
using namespace cv;
using namespace cv_bridge;

//ros::Publisher	                red_pub_;
//image_transport::Publisher 		image_pub_;

//static const std::string OPENCV_WINDOW 	= "Image window";
//static const std::string OUT_WINDOW 	= "Red Filter";

static const int LIMIT = 10000;
//int pix_counter = 0;

CvImagePtr generate_cv_ptr(sensor_msgs::ImageConstPtr msg) {
    CvImagePtr cv_ptr;
    
    try {
	    cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(EXIT_FAILURE);
    }
    return cv_ptr;
}


/*Vec3b pix_fxn(Vec3b color) {
    int r = color[2];
	int g = color[1];
	int b = color[0];
				
	if(r >= 110 && g <= b && r > g) {
		pix_counter++;
	} else {
		color = Vec3b(0,0,0);
	}
    return color;
} 
*/
Mat process_image(CvImagePtr cv_ptr, Vec3b (*process_pixel)(Vec3b, int)) {
    Mat outImg = cv_ptr->image.clone();
    Size s = cv_ptr->image.size();
    int pix_counter = 0;
    for(int row = 0; row < s.height; row++) {
	    for(int col = 0; col < s.width; col++) {
			Vec3b color = (cv_ptr->image).at<Vec3b>(Point(col, row));
            outImg.at<Vec3b>(Point(col, row)) = (*process_pixel)(color, col);
		}
    }
    return outImg;
}
/*
class ImageConverter
{
	ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber 	image_sub_;
 
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
		destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
        CvImagePtr cv_ptr = generate_cv_ptr(msg);   
		
        Mat outImg = process_image(cv_ptr, pix_fxn); 
    
		std_msgs::Bool is_red;
		is_red.data = false;
		if (pix_counter > LIMIT) {
			is_red.data = true;
		}
		
		red_pub_.publish(is_red);

		imshow(OUT_WINDOW, outImg);
		waitKey(3);
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
*/
