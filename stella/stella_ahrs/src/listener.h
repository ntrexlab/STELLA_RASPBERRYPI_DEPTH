#include <thread>
#include <ros/ros.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sensor_msgs/Imu.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <pthread.h>
#include "MwSerial.hpp"


#define DEG2RAD( a ) ( (a) * (M_PI/180.0f) )
#define COS(a) cos(DEG2RAD(a))
#define SIN(a) sin(DEG2RAD(a))

#define DeviceID 0x01
#define STX 0x02
#define ETX 0x03
#define Command 0xF0

#define ACC 0x33
#define GYO 0x34
#define DEG 0x35


using namespace std;

char data[8];
long id = 0;
int length = 0;
bool run = true;

int16_t acc_x = 0, acc_y = 0, acc_z = 0, gyo_x = 0, gyo_y = 0, gyo_z = 0, deg_x = 0, deg_y = 0, deg_z = 0; 

sensor_msgs::Imu imu;
