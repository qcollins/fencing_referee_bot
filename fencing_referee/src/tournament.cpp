#include <cmath> 
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
using namespace std;

/* Message integer constants */
static const int NONE  = 0;
static const int RED   = 1;
static const int GREEN = 2;
static const int BOTH  = 3;

int score = NONE;

struct tableCell {
	int name_1_score;
	int name_2_score;	
};

void scoreCB(std_msgs::Int8 light_score)
{
	score = light_score.data;
}

tableCell bout(int max_score, int name_1_index, int name_2_index) 
{
	cout<<"BIIIISH";
	int fencer1_score = 0;
	int fencer2_score = 0;
	
	ros::Rate loop_rate(60); //60 Hz
	
	// Start timer
	
	while(ros::ok()) {
		ros::spinOnce();

		if (score == RED) {
			fencer1_score++;
		} else if (score == GREEN) {
			fencer2_score++;
		} else if (score == BOTH) {
			fencer1_score++;
			fencer2_score++;
		}
		
		if (fencer1_score == max_score || fencer2_score == max_score) {
			tableCell result;
			result.name_1_score = fencer1_score;
			result.name_2_score = fencer2_score;
			
			return result;
		}
		
		loop_rate.sleep();
	}
}

int main(int argc, char **argv) {
		/* Initializing our node */
		ros::init(argc, argv, "tournament");
	
		/* Subscribers & Publishers */
		ros::NodeHandle nh_;
		ros::Subscriber score_sub = nh_.subscribe("/score", 1, scoreCB);	
			
		/* Initializing variables */
		ros::Rate loop_rate(60); //60 Hz
			
		/* Initialize fencers */
		int numFencers, numRounds;
		cout << "Number of fencers: ";
		cin >> numFencers;
		
		numRounds = numFencers * (numFencers - 1);
		string names[numFencers];
		int table[numFencers][numFencers];
		
		for (int i = 0; i < numFencers; i++) {
			string name;
			cout << "Name of fencer " << i + 1 << ": ";
			cin >> name;
			names[i] = name;
			table[i][i] = 0;
		}
		
		pair table_of_4[] = {};
		pair table_of_5[] = {(1,4), (2,5), (3,1)};
		
		tableCell result;
		for (int i = 0; i < numRounds; i++) {
				result = bout(5, 1st, 2nd);
				table[1st][2nd] = result.name_1_score;
				table[2nd][1st] = result.name_2_score;
		}
		
		calculate_result(table);
				
		return 0;
}
