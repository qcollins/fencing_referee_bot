#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Publisher velocity_pub;

const int CLOCK         = 1;
const int COUNTER_CLOCK = -1;

const float SPEED = 0.6;
const float RAD   = 0.255;

int DIRECTION = 0;

void rotate()
{
        ros::Rate loop_rate(60); //60 Hz
        geometry_msgs::Twist vel_msg;      
        vel_msg.angular.z = SPEED * DIRECTION;
        float angle       = RAD * DIRECTION;

        double curr_angle = 0.0;
        double init_time  = ros::Time::now().toSec();
        do {
                velocity_pub.publish(vel_msg);
                double curr_time = ros::Time::now().toSec();
                curr_angle = SPEED * (curr_time - init_time);
                ros::spinOnce();
                loop_rate.sleep();
        } while (curr_angle < angle);
        
        vel_msg.angular.z = 0;
        velocity_pub.publish(vel_msg);
}

void pctCB(std_msgs::Float32 relative_pos)
{
        if (relative_pos.data > 10) {
                DIRECTION = -1;
        } else if (relative_pos.data < -10) {
                DIRECTION = 1;
        } else {
				DIRECTION = 0;
		}
}

int main(int argc, char** argv) 
{

        ros::init(argc, argv, "camera_rotate");
        ros::NodeHandle nh_;
        
        velocity_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        ros::Subscriber fencer_sub = nh_.subscribe("/avg_white", 1, pctCB);
        
        while(ros::ok()) {
                //ros::spinOnce();
                rotate();  
        }

        return 0;
}
