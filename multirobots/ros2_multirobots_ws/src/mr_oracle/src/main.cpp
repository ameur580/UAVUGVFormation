#include <iostream>
#include "mr_aggregator/aggregator.hpp"
#include "mr_oracle/averager.hpp"


void signal_handler(int signal){
	cout << signal << endl; 
	raise(SIGKILL);
}


int main(int argc, char *argv[]){
	signal(SIGINT, signal_handler);
	new Aggregator<Averager>(argc, argv);
	return 0;
}
