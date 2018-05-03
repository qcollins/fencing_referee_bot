#include <stdlib.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char * argv[]) {
    int n,m,i=1;
    ifstream inFile;
    inFile.open("/home/turtlebot/catkin_ws/src/fencing_referee_bot/fencing_referee/libs/pairs/pairs_3.txt");
    if(!inFile) {
        cerr << "Failed to open file";
        exit(1);
    }
    
    while (inFile >> i) {
        printf("%d ", i);
    }
    inFile.close();
    return 0;
}
