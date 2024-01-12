#include <csignal>
#include "epuck_controller/control.hpp"
#include "epuck_controller/control/shared.hpp"

using namespace std;

extern shared_t shared;
using namespace epuck_controller;

Control *control;
void signal_handler(int signal)
{
	cout << signal << endl;
	std::raise(SIGKILL);
}

int main(int argc, char *argv[]){

	if (argc<2) exit(1);


	control = new Control(argc,argv);

	std::signal(SIGINT, signal_handler);

	shared.xtargetg = -1.0;
	shared.ytargetg = -1.0;
	if (argc>2) shared.xtargetg = atof(argv[2]);
	if (argc>3) shared.ytargetg = atof(argv[3]);

	shared.state = TURN_TOWARDS_TARGET;
	shared.v = 2; 
	shared.vl = -shared.v;
	shared.vr = -shared.vl;

	control->go();

}
