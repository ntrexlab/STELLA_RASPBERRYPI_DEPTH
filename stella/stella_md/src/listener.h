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
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//



#define DEG2RAD( a ) ( (a) * (M_PI/180.0f) )
#define COS(a) cos(DEG2RAD(a))
#define SIN(a) sin(DEG2RAD(a))


using namespace std;

float imu_data[8] = {0,};
char read_buf [256];
char buff [20];
int serial_port;

char write_buf [256];
char *sArr[2] = {0,};


sensor_msgs::Imu imu;


ros::Time current_time, last_time;

float left_rpm = 0, right_rpm = 0;
int left_encoder = 0, right_encoder = 0,delta_left = 0,delta_right = 0,left_encoder_prev=0,right_encoder_prev=0;

float linear_x = 0.0, angular_ = 0.0;

double delta_th=0.0,delta_s=0.0,delta_x=0.0,delta_y=0.0,x=0.0,y=0.0,th=0.0;

class ntrex_can_fifo
{
    private:
    ros::Publisher chatter_pub;
    ros::Subscriber sub;

    ros::NodeHandle n;

    std::thread* thread_read_AHRS;
    std::thread* thread_pub_odm;

    tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
   
    public:

    ~ntrex_can_fifo();
    ntrex_can_fifo();

    void readStatus();
    void writepub();
    void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void MD_input(char* str);

    void run();
};
