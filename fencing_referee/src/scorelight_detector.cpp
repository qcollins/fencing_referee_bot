#include <cmath> 
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
using namespace std;

/* Message integer constants */
static const int NONE  = 0;
static const int RED   = 1;
static const int GREEN = 2;
static const int BOTH  = 3;

/* Color initializations */
int red   = 0;
int green = 0;

/* Red callback - sets red = 1 if red is detected */
void redCB(std_msgs::Bool is_red)
{
	if(is_red.data == true) {
		red = 1;
	} else {
		red = 0;
	}
}

/* Green callback - sets green = 1 if green is detected */
void greenCB(std_msgs::Bool is_green)
{
	if(is_green.data == true) {
		green = 1;
	} else {
		green = 0;
	}
}

int main(int argc, char **argv) {
		/* Initializing our node */
		ros::init(argc, argv, "scorelight_detector");
	
		/* Subscribers & Publishers */
		ros::NodeHandle nh_;
		ros::Publisher  score_pub = nh_.advertise<std_msgs::Int8>("score", 1);
		ros::Subscriber green_sub = nh_.subscribe("/green", 1, greenCB);
		ros::Subscriber red_sub   = nh_.subscribe("/red",   1, redCB);
		
		/* Initializing variables */
		ros::Rate loop_rate(60); //60 Hz
		
		double red_timer   = 0.0;
		double green_timer = 0.0;
		
		std_msgs::Int8 score;
		score.data = NONE;
		double timer = 0.0;

		/* Begin detection of colors; pauses if color is detected */
		while(ros::ok()) {
			ros::spinOnce();
			
			bool red_s   = false;
			bool green_s = false;
			bool update_time = false;
			
			if (timer < ros::Time::now().toSec()) {
				/* If red is detected */
				if (red == 1) {
					red_s = true;
					update_time = true;
				}
				
				/* If green is detected */
				if (green == 1) {
					green_s = true;
					update_time = true;
				}
				
				/* Pause detection if either/both color(s) are detected */
				if (update_time) {
					timer = ros::Time::now().toSec() + 3;
				}
			}
			
			/* Assign relevant data to Int msg */
			if (red_s == true && green_s == true) {
				score.data = BOTH;
			} else if (red_s == false && green_s == true) { 
				score.data = GREEN;
			} else if (red_s == true && green_s == false) {
				score.data = RED;
			} 
			
			/* Publish if there is data */
			if (score.data != NONE) {
				score_pub.publish(score);
			}
			score.data = NONE;
			
			/* Sleep */
			loop_rate.sleep();
		}
			
		return 0;
}
