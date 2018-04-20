#include <algorithm>
#include <cmath> 
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>
using namespace std;

string to_str(int i) {
	stringstream ss;
	ss << i;
	return ss.str();
}


/*  ============= VARIABLES  ============= */
/* Message integer constants */
static const int NONE  = 0;
static const int RED   = 1;
static const int GREEN = 2;
static const int BOTH  = 3;

/* Score constants */
static const int MAX_SCORE = 5;
			 int score     = NONE;
			 
/*  ============= FUNCTIONS  ============= */
/*  --------------- ROS FUNCTIONS --------------- */
/* Score callback function */
void scoreCB(std_msgs::Int8 light_score)
{
	score = light_score.data;
}

/* Text-to-speech function */
void say_string(ros::Publisher sound_pub, string text) 
{
	sound_play::SoundRequest S;
	
	S.sound = -3;
	S.command = 1;
	S.arg = text;
	
	sound_pub.publish(S);
}

/*  --------------- INIT FUNCTIONS --------------- */
/* 
 * Stores the number of fencers
 */
int get_num_fencers() 
{
		int numFencers;
		
		cout << "Number of fencers (3 to 7): ";
		cin >> numFencers;
		while (numFencers < 3 || numFencers > 7) {
			cout << "Invalid number. Number of fencers (3 to 7): ";
			cin >> numFencers;
		}
		
		return numFencers;
}

/* 
 * Returns the table order based on the number of fencers in the tournament
 */ 
vector<pair<int, int> > get_table(int numFencers)
{
		vector<pair<int, int> > table;
		
		if (numFencers == 3) {
			table.push_back(make_pair(0,1));
			table.push_back(make_pair(1,2));
			table.push_back(make_pair(0,2));
		} else if (numFencers == 4) {
			table.push_back(make_pair(0,3));
			table.push_back(make_pair(1,2));
			table.push_back(make_pair(0,2));
			table.push_back(make_pair(1,3));
			table.push_back(make_pair(2,3));
			table.push_back(make_pair(0,1));
		} else if (numFencers == 5) {
			table.push_back(make_pair(0,1));
			table.push_back(make_pair(2,3));
			table.push_back(make_pair(4,0));
			table.push_back(make_pair(1,2));
			table.push_back(make_pair(4,3));
			table.push_back(make_pair(0,2));
			table.push_back(make_pair(1,4));
			table.push_back(make_pair(3,0));
			table.push_back(make_pair(2,4));
			table.push_back(make_pair(3,1));
		} else if (numFencers == 6) {
			table.push_back(make_pair(0,1));
			table.push_back(make_pair(3,4));
			table.push_back(make_pair(1,2));
			table.push_back(make_pair(4,5));
			table.push_back(make_pair(2,0));
			table.push_back(make_pair(5,2));
			table.push_back(make_pair(1,4));
			table.push_back(make_pair(0,3));
			table.push_back(make_pair(4,2));
			table.push_back(make_pair(0,5));
			table.push_back(make_pair(3,1));
			table.push_back(make_pair(2,5));
			table.push_back(make_pair(4,0));
			table.push_back(make_pair(2,3));
			table.push_back(make_pair(5,1));
		} else { // numFencers == 7
			table.push_back(make_pair(0,3));
			table.push_back(make_pair(1,4));
			table.push_back(make_pair(2,5));
			table.push_back(make_pair(6,0));
			table.push_back(make_pair(4,3));
			table.push_back(make_pair(1,2));
			table.push_back(make_pair(5,6));
			table.push_back(make_pair(4,0));
			table.push_back(make_pair(3,2));
			table.push_back(make_pair(5,1));
			table.push_back(make_pair(4,6));
			table.push_back(make_pair(2,0));
			table.push_back(make_pair(3,5));
			table.push_back(make_pair(6,1));
			table.push_back(make_pair(2,4));
			table.push_back(make_pair(0,5));
			table.push_back(make_pair(1,3));
			table.push_back(make_pair(6,2));
			table.push_back(make_pair(5,4));
			table.push_back(make_pair(0,1));
			table.push_back(make_pair(3,6));
		}
		
		return table;									  
}

/*  --------------- BOUT FUNCTIONS --------------- */
/* 
 * Commences individual bouts
 */
pair<int, int> bout(int max_score, int name_1_index, int name_2_index) 
{
	int fencer1_score = 0;
	int fencer2_score = 0;
	
	string start;
	cout << "Type 'Y' to start: ";
	cin >> start;
	
	ros::Rate loop_rate(60); //60 Hz
		
	while(ros::ok()) {
		ros::spinOnce();

		if (score == RED) {
			fencer1_score++;
			ROS_INFO("Score: %d vs. %d", fencer1_score, fencer2_score);
		} else if (score == GREEN) {
			fencer2_score++;
			ROS_INFO("Score: %d vs. %d", fencer1_score, fencer2_score);
		} else if (score == BOTH) {
			fencer1_score++;
			fencer2_score++;
			ROS_INFO("Score: %d vs. %d", fencer1_score, fencer2_score);
		}
		
		score = NONE;
			
		if (fencer1_score == max_score && fencer2_score == max_score) {
			fencer1_score--;
			fencer2_score--;
		} else if (fencer1_score == max_score || fencer2_score == max_score) {
			pair<int, int> result;
			result.first = fencer1_score;
			result.second = fencer2_score;
			
			return result;
		}
		
		loop_rate.sleep();
	}
}

/* 
 * Commences the entire tournament
 */ 
vector <vector <int> > commence_tournament(string names[],
									       vector<vector <int> >   table, 
										   vector<pair<int, int> > pool,
										   int numFencers, 
										   ros::Publisher sound_pub) 
{
	for (int i = 0; i < numFencers; i++) {
		table[i][i] = 0;
	}
	
	int numRounds = (numFencers * (numFencers - 1)) / 2;
	for (int i = 0; i < numRounds; i++) {
		int fencer1 = pool[i].first;
		int fencer2 = pool[i].second;
		string call_fencers = "Bout "+ to_str(i+1) +": fencers "+names[fencer1]+" and "+names[fencer2]+" to fence.";
		say_string(sound_pub, call_fencers);
		ROS_INFO("Bout %d: Fencers %s and %s to fence.", 
				  i + 1, names[fencer1].c_str(), names[fencer2].c_str());
		
		pair<int, int> result = bout(MAX_SCORE, fencer1, fencer2);
		
		table[fencer1][fencer2] = result.first;
		table[fencer2][fencer1] = result.second;
		
		cout << names[fencer1] << ": " << table[fencer1][fencer2] << endl
			 << names[fencer2] << ": " << table[fencer2][fencer1] << endl;
	}
	
	return table;	
}

/*  --------------- CALC FUNCTIONS --------------- */
/* Indicator comparison function */
bool comp_indicator(pair<string, pair<int, int> > a, pair<string, pair<int, int> > b) 
{
	return a.second.second > b.second.second;
}

/* Victory comparison function */
bool comp_victory(pair<string, pair<int, int> > a, pair<string, pair<int, int> > b) 
{
	return a.second.first > b.second.first;
} 

/* 
 * Calculates the results (victories/indicator) of a single fencer 
 */ 
pair<string, pair<int, int> > calculate_result(string names[], 
											   vector<vector <int> > table, 
											   int i, int numFencers)
{
	int victories = 0;
	int indicator = 0;
	
	for (int j = 0; j < numFencers; j++) {
		if (table[i][j] == MAX_SCORE) {
			victories++;
		}
		indicator += (table[i][j] - table[j][i]);
	}
	
	return (make_pair(names[i], make_pair(victories, indicator)));
}

/* 
 * Output/Result-printing helper function
 */ 
void output_list(vector<pair<string, pair<int, int> > > result_list, int numFencers)
{	
	cout << "\nThe results are: \n";
	for (int i = 0; i < numFencers; i++) {
		cout << i + 1 << ". "   << result_list[i].first << "\t"
				      << "(V: " << result_list[i].second.first << ", \t"
				      << "I: "  << result_list[i].second.second << ")\n";
	}
}

/* 
 * Calculates the overall standings in the tournament, and outputs 
 * the results
 */ 
void calculate_results(string names[], vector<vector <int> > table,
					  int numFencers)
{
	vector<pair<string, pair<int, int> > > result_list;
	
	/* Calculate results */
	for (int i = 0; i < numFencers; i++) {
		result_list.push_back(calculate_result(names, table, i, numFencers));
	}
	
	/* Sort results */
	stable_sort(result_list.begin(), result_list.end(), comp_indicator);
	stable_sort(result_list.begin(), result_list.end(), comp_victory);
	
	/* Output results */
	output_list(result_list, numFencers);
}

/*  --------------- MAIN --------------- */
int main(int argc, char **argv) 
{
		/* Initializing our node */
		ros::init(argc, argv, "tournament");
	
		/* Subscribers & Publishers */
		ros::NodeHandle nh_;
		ros::Subscriber score_sub = nh_.subscribe("/score", 1, scoreCB);
		ros::Publisher  sound_pub = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1);
		ros::Rate loop_rate(60); 	
					
		/* Initializing fencers */
		int numFencers = get_num_fencers();
		
		/* Creating tournament table */
		string 		   		   names[numFencers];
		vector <vector <int> > table(numFencers, vector<int>(numFencers));
		
		for (int i = 0; i < numFencers; i++) {
			string name;
			cout << "Name of fencer " << i + 1 << ": ";
			cin >> name;
			names[i] = name;
		}
			
		/* Determine the pool order to use */
		vector<pair<int, int> > pool = get_table(numFencers);
		
		/* Commence the tournament */
		table = commence_tournament(names, table, pool, numFencers, sound_pub);
			
		/* Calculate the results of the bout, and output results */
		calculate_results(names, table, numFencers);
		
		return 0;
}
