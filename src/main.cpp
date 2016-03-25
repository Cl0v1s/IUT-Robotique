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

static void moveBackLeg(int leg, double dephas, double t)
{
    double shoulder = 0;
    double rotula = 0.3*sin(20000*t+dephas)-0.3;
    double finger = 0;
    VREP.getMotor(leg).writePos(shoulder);
    VREP.getMotor(leg+1).writePos(rotula);
    VREP.getMotor(leg+2).writePos(finger);
}


static void moveMotorsStep(double t)
{
    fixLeg(0);
    fixLeg(3);
    fixLeg(6);
    fixLeg(9);
    fixLeg(12);
    fixLeg(15);

    moveFrontLeg(0,0,t);
    moveFrontLeg(3,M_PI/3,t);
    //moveSideLeg(6,0,t);
    moveFrontLeg(9,M_PI/2,t);
    moveFrontLeg(12,M_PI,t);
    //moveSideLeg(15,M_PI,t);


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
