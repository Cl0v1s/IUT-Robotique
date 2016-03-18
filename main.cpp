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

static double matrix [4][6][3] =
{
    {

        {0, 0,0},
        {0, -0.2,0},
        {0, 0.2,0},
        {0, -0.2, 0},
        {0, 0,0},
        {0, 0.2,0},
    },

    {

        {0, 0,0},
        {0.6, -0.2,0},
        {0, 0.2,0},
        {-0.6, -0.2, 0},
        {0, 0,0},
        {0, 0.2,0},
    },

    {

        {0.6, -0.2,0},
        {0.6, 0,0},//
        {0, -0.2,0},
        {-0.6, 0, 0},//
        {-0.6, -0.2,0},
        {0, -0.2,0},
    },

    {

        {-0.6, 0,0},
        {-0.6, 0,0},//
        {0, -0.2,0},
        {0.6, 0, 0},//
        {0.6, 0,0},
        {0, -0.2,0},
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
/*
static void moveSideLeg(int leg,double dephas,double t)
{
    double shoulder = sin(2000*t + dephas);
    double rotula = sin(2000*t + M_PI/2);

    VREP.getMotor(leg).writePos(shoulder);
    VREP.getMotor(leg+1).writePos(rotula);
    VREP.getMotor(leg+2).writePos(0);
}

static void fixLeg(int leg)
{
    VREP.getMotor(leg).writePos(0);
    VREP.getMotor(leg+1).writePos(0);
    VREP.getMotor(leg+2).writePos(0);
}

static void moveFrontLeg(int leg, double dephas, double t)
{
    double shoulder = 0;
    double rotula = sin(2000*t+dephas);
    double finger = sin(2000*t+dephas)+0.2;
    VREP.getMotor(leg).writePos(shoulder);
    VREP.getMotor(leg+1).writePos(rotula);
    VREP.getMotor(leg+2).writePos(finger);
}
/*
static void moveBackLeg(int leg, double dephas, double t)
{
    double shoulder = 0;
    double rotula = 0.3*sin(20000*t+dephas)-0.3;
    double finger = 0;
    VREP.getMotor(leg).writePos(shoulder);
    VREP.getMotor(leg+1).writePos(rotula);
    VREP.getMotor(leg+2).writePos(finger);
}
*/

static void applyMatrix(double t, int frame)
{
    for(int i = 0; i != 6;i++)
    {
        /*
        VREP.getMotor(i*3).writePos(*((matrix+i*3) + 0));
        VREP.getMotor((i*3)+1).writePos(*((matrix+i*3) + 1));
        VREP.getMotor((i*3)+2).writePos(*((matrix+i*3) + 2));
        */
    }
}

static void moveMotorsStep(double t)
{
    int frame = t*4;
    frame = frame%4;

    int offset = t / frame * frame;

    double ti0 = frame + offset;
    double ti1 = (frame + 1) + offset;

    for (int i = 0; i != 6; i++)
    {
        double new_pos0 = (matrix[frame][i][0]) + (((t-ti0)/(ti1-ti0)) * ((matrix[frame+1][i][0]) - (matrix[frame][i][0])));
        double new_pos1 = (matrix[frame][i][1]) + (((t-ti0)/(ti1-ti0)) * ((matrix[frame+1][i][1]) - (matrix[frame][i][1])));
        double new_pos2 = (matrix[frame][i][2]) + (((t-ti0)/(ti1-ti0)) * ((matrix[frame+1][i][2]) - (matrix[frame][i][2])));
        VREP.getMotor(i*3).writePos(new_pos0);
        VREP.getMotor((i*3)+1).writePos(new_pos1);
        VREP.getMotor((i*3)+2).writePos(new_pos2);
    }


/*
    fixLeg(12);
    fixLeg(15);


    //Right Middle
    moveSideLeg(0,0,t);
    //Left Middle
    moveSideLeg(9,M_PI,t);
    //Front Left
    moveFrontLeg(3,M_PI/2.5,t);
    //Front Right
    moveFrontLeg(6,M_PI/2.5,t);

    //moveBackLeg(12,M_PI,t);
    //moveBackLeg(15,0, t);
*/
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
