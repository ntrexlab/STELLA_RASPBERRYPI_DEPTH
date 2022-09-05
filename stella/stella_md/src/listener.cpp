#include "listener.h"
#include "mobilerobot.h"

ntrex_can_fifo::ntrex_can_fifo()
{
    chatter_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    sub = n.subscribe("cmd_vel", 10, &ntrex_can_fifo::chatterCallback, this);

    thread_read_AHRS = new std::thread(&ntrex_can_fifo::readStatus, this);
}

ntrex_can_fifo::~ntrex_can_fifo()
{
    thread_read_AHRS->join();

    delete thread_read_AHRS;
    close(serial_port);
}

void ntrex_can_fifo::MD_input(char *str)
{
    if (!strcmp(str, "move"))
    {
        sprintf(write_buf, "mvc=%0.3f,%0.3f\r\n", right_rpm, left_rpm); //

        for (int i = 0; i < strlen(write_buf); i++)
        {
            unsigned char buf = (unsigned char)(write_buf[i]);
            write(serial_port, &buf, sizeof(buf));
        }
    }

    if (!strcmp(str, "encoder"))
    {
        sprintf(write_buf, "mp\r\n");

        for (int i = 0; i < strlen(write_buf); i++)
        {
            unsigned char buf = (unsigned char)(write_buf[i]);
            write(serial_port, &buf, sizeof(buf));
        }
    }
}

void ntrex_can_fifo::chatterCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    calculate_wheel_vel(msg->linear.x, msg->angular.z, &left_rpm, &right_rpm);

    linear_x = msg->linear.x;
    angular_ = msg->angular.z;

    MD_input("move");
}

void ntrex_can_fifo::readStatus()
{
    ros::Rate rate(20);

    while (1)
    {
        MD_input("encoder");

        current_time = ros::Time::now();

        int i = 0;
        char parsing[3][20];

        int nbytes = read(serial_port, &read_buf, sizeof(read_buf));

        if (nbytes > 0)
        {
            memcpy(buff,&read_buf[0],3);

            char *ptr = strtok(read_buf + 3, ",");

            i = 0;

            while (ptr != NULL)
            {
                strcpy(parsing[i++], ptr);
                ptr = strtok(NULL, ",");
            }

            if(strcmp(buff,"mp=") == 0)
            {
              left_encoder = atoi(parsing[1]);
              right_encoder = atoi(parsing[0]);
            }

            delta_left = (left_encoder - left_encoder_prev) * -1;
            delta_right = (right_encoder - right_encoder_prev) * -1;

            if (abs(delta_left) < 12000 && abs(delta_right) < 12000)
            {
                delta_s = (delta_left + delta_right) / 2.0 / pulse_per_distance;
                delta_th = ((delta_right - delta_left) / wheel_to_wheel_d / pulse_per_distance);
                delta_x = (delta_s * cos(th + delta_th));
                delta_y = (delta_s * sin(th + delta_th));
            }

            x += delta_x;
            y += delta_y;
            th += delta_th;
        }

        geometry_msgs::Quaternion Quaternion = tf::createQuaternionMsgFromYaw(th);
   
        transform.setOrigin( tf::Vector3(x, y,0));
        transform.setRotation(tf::Quaternion(Quaternion.x,Quaternion.y,Quaternion.z,Quaternion.w));

        odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));

        nav_msgs::Odometry odom;

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = Quaternion;

        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = linear_x;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = angular_;

        chatter_pub.publish(odom);
        left_encoder_prev = left_encoder;
        right_encoder_prev = right_encoder;

        rate.sleep();
        last_time = current_time;
    }
}

void ntrex_can_fifo::run()
{
    ros::Rate rate(1000);
    ros::spin();

    while (ros::ok())
    {
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stella_mw_driver_node");
    
    serial_port = open("/dev/MW", O_RDWR);

    struct termios tty;

    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    ntrex_can_fifo node;

    node.run();

    return 0;
}
