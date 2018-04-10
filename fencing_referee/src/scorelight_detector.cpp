#include <ros/ros.h>
#include <cmath> 
#include <std_msgs/Bool.h>
using namespace std;

static const int RED   = 1;
static const int GREEN = 2;
static const int BOTH  = 3;

int red = 0;

void redCB(std_msgs::Bool is_red)
{
	if(is_red.data == true) {
		red = 1;
		//ROS_INFO("FUCK\n");
	} else {
		red = 0;
	}
	
	/*else if(is_red.data == false && RED_COUNT > 0) {
		red = 0;
		RED_COUNT --;
	}
	
	if(RED_COUNT >= 15) {
		RED_COUNT = 0;
		
	}*/
}

/*void greenCB(std_msgs::Bool is_green){
	if(is_red.data == True) {
		RED_COUNT ++;
	}
}*/


int main(int argc, char **argv) {
		/* Initializing our node */
		ros::init(argc, argv, "scorelight_detector");
	
		/* Subscribers & Publishers */
		ros::NodeHandle nh_;
		//ros::Publisher  score_pub = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
		//ros::Subscriber green_sub = nh_.subscribe("/green", 100, greenCB);
		ros::Subscriber red_sub   = nh_.subscribe("/red",   1, redCB);
	
		//if (green_sub.output = true) {
			// ...
		//}
		
		/* publish velocity commands forward until we move the desired distance */
		ros::Rate loop_rate(60); //60 Hz
		
		//int    red_on = 0;	
		double red_timer = 0.0;
		
		while(ros::ok()) {
			ros::spinOnce();
			if (red == 1 && ros::Time::now().toSec() > red_timer) {
				red_timer = ros::Time::now().toSec() + 3;
				// increase score
				ROS_INFO("Hello\n");
			}
			loop_rate.sleep();
		}
				
				
			/*	
				if (red_on == 0) {
					// increase score
					red_on++;
					// sleep subscriber
				} else if (red_on > 0 && red_on < 15) {
					red_on++;
				} else { //(red_on > 15) if red is on for >15s
					red_on = 0;
				}				
			} else {
				// do nothing
			}
		}*/
	
		/*bool red   = false;
		bool green = false;
		std_msgs::Int8 score;*/
		//...code		
		/* 
		 * if (red flashes for 15 cycles)
		 * 		red = true;
		 * 		sleep subscriber for 5 sec
		 * if (green flashes for 15 cycles)
		 * 		green = true;
		 * 		sleep subscriber for 5 sec
		 * 
		 * 
		 */
		/*
		if (red == true && green == true) { // i.e. both lights flash
			score.data = BOTH;
		} else if (green == true) {
			score.data = GREEN;
		} else { // red == true 
			score.data = RED;
		}
		score_pub.publish(score);
		*/	
	
	return 0;
}

