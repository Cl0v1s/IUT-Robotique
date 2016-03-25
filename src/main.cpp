#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "VREPClient.hpp"

/**
 * VREPClient instance
 * Globale variable
 */
static VREPClient VREP;

static double FRAMES [4][6][3] =
{
   {
        {-0.6, 0,0},
        {0, 0,-1},
        {0.6, 0,0},
        {-0.6, 0.5,-0.5},
        {0, 0,0},
        {-0.6, 0.5,-0.5},
    },

    {

        {0.6, 0,0},
        {0, 1,0},
        {-0.6, 0,0},
        {-0.6, 0.5,-0.5},
        {0, 0.3,-1},
        {-0.6, 0.5,-0.5},
    },

    {

        {0.6, 0.5,-0.5},
        {0, 0,0},//
        {-0.6, 0.5,-0.5},
        {-0.6, 0.5,-0.5},
        {0, 0,0},
        {-0.6, 0.5,-0.5},
    },

    {

        {-0.6, 0.5,-0.5},
        {-0, 0,0},//
        {0.6, 0.5,-0.5},
       {-0.6, 0.5,-0.5},
        {0, 0,0},
        {-0.6, 0.5,-0.5},
    },
};

using namespace std;

static void exiting(bool success = true)
{
    //Close server connection
    VREP.stop();
    VREP.disconnect();
    if (success) {
        exit(EXIT_SUCCESS);
    } else {
        exit(EXIT_FAILURE);
    }
}

static void signal_handler(int sig, siginfo_t *siginfo, void *context)
{
    cout << endl << "Exiting..." << endl;
    exiting();
}

static void attachSignalHandler()
{
    struct sigaction action;
    bzero(&action, sizeof(action));
    action.sa_sigaction = &signal_handler;
    action.sa_flags = SA_SIGINFO;
    if (sigaction(SIGINT, &action, NULL) < 0) {
        cerr << "Unable to register signal handler" << endl;
        exit(EXIT_FAILURE);
    }
}

static void displayInitialState()
{
    //Display founded motors
    size_t countMotors = VREP.countMotors();
    cout << "Registered motors: " << countMotors << endl;
    for (size_t i=0;i<countMotors;i++) {
        cout << "[" << i << "] ";
        cout << VREP.getMotor(i).getName() << " ";
        cout << "minPos=" << VREP.getMotor(i).getMinPos() << " ";
        cout << "maxPos=" << VREP.getMotor(i).getMaxPos() << " ";
        cout << "TorqueMax=" << VREP.getMotor(i).getTorqueMax() << endl;
    }
    //And force sensors
    size_t countForceSensors = VREP.countForceSensors();
    cout << "Registered force sensors: " << countForceSensors << endl;
    for (size_t i=0;i<countForceSensors;i++) {
        cout << "[" << i << "] ";
        cout << VREP.getForceSensor(i).getName() << endl;
    }
}

static void displayState()
{
    size_t countMotors = VREP.countMotors();
    //Display motor states
    for (size_t i=0;i<countMotors;i++) {
        cout << "   #[" << i << "] ";
        cout << VREP.getMotor(i).getName() << " ";
        cout << "pos=" << VREP.getMotor(i).readPos() << " ";
        cout << "torque=" << VREP.getMotor(i).readTorque() << endl;
    }
    size_t countForceSensors = VREP.countForceSensors();
    //Display force sensors value
    for (size_t i=0;i<countForceSensors;i++) {
        cout << "   *[" << i << "] ";
        cout << VREP.getForceSensor(i).getName() << " ";
        cout << "force=" << VREP.getForceSensor(i).readForceNorm() << " ";
        cout << "torque=" << VREP.getForceSensor(i).readTorqueNorm() << endl;
    }
    //Display accelerometer sensor
    cout << "   -    Accelerometer ";
    cout << "X=" << VREP.readAccelerometerX() << " ";
    cout << "Y=" << VREP.readAccelerometerY() << " ";
    cout << "Z=" << VREP.readAccelerometerZ() << endl;
    //Display position tracker
    cout << "   -    Position tracker ";
    cout << "X=" << VREP.readPositionTrackerX() << " ";
    cout << "Y=" << VREP.readPositionTrackerY() << " ";
    cout << "Z=" << VREP.readPositionTrackerZ() << endl;
}

static void applyFRAMES(double t)
{
	int frame = t;
	frame = frame%4;
	for(int i = 0; i != 6;i++)
	{
		/*double new_pos0 = (FRAMES[frame][i][0]) + (((t-ti0)/(ti1-ti0)) * ((FRAMES[frame+1][i][0]) - (FRAMES[frame][i][0])));
		double new_pos1 = (FRAMES[frame][i][1]) + (((t-ti0)/(ti1-ti0)) * ((FRAMES[frame+1][i][1]) - (FRAMES[frame][i][1])));
		double new_pos2 = (FRAMES[frame][i][2]) + (((t-ti0)/(ti1-ti0)) * ((FRAMES[frame+1][i][2]) - (FRAMES[frame][i][2])));*/
        double alpha = t - ((int)t);
        float distanceY1 = FRAMES[frame+1][i][0] - FRAMES[frame][i][0];
        float distanceY2 = FRAMES[frame+1][i][1] - FRAMES[frame][i][1];
        float distanceY3 = FRAMES[frame+1][i][2] - FRAMES[frame][i][2];

		VREP.getMotor(i*3).writePos(FRAMES[frame][i][0] + distanceY1*alpha);
		VREP.getMotor((i*3)+1).writePos(FRAMES[frame][i][1] +distanceY2*alpha);
		VREP.getMotor((i*3)+2).writePos(FRAMES[frame][i][2] +distanceY3*alpha);
	}
}

static void moveMotorsStep(double t)
{
	applyFRAMES(t);
}




static double degToRad(double deg)
{
    return deg*(M_PI/180);
}

static double radToDeg(double rad)
{
    return (180/M_PI)*rad;
}

int main(int argc, char* argv[])
{
    //Network parameters
    int port = 0;
    char* ip = NULL;

    //Parse input arguments
    if (argc != 3) {
        cerr << "Bad usage. Usage: ./command [ip address] [port number]" << endl;
        cerr << "Provide network parameters to connect to V-REP server" << endl;
        return EXIT_FAILURE;
    } else {
        ip = argv[1];
        port = atoi(argv[2]);
    }

    //Signal attaching
    attachSignalHandler();

    try {
        //Connection to V-REP
        cout << "Connecting to V-REP server " << ip << ":" << port << endl;
        VREP.connect(ip, port);
        //Display initial state
        displayInitialState();
        //Main Loop
        cout << "Starting simulation" << endl;
        VREP.start();
        for (double t=0;t<60.0;t+=0.050) {
            //Display state
            cout << "Simulation step t=" << t << endl;
            displayState();
            //Do next step
            VREP.nextStep();
            //Compute motors move
            moveMotorsStep(t);
        }
        //End simulation
        cout << "Stopping simulation" << endl;
        VREP.stop();
    } catch (string str) {
        cerr << "Exception error: " << str << endl;
        exiting(false);
    }

    exiting();
}
