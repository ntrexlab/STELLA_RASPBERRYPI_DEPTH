#include "listener.h"

void *AHRS_thread(void *arg)
{
    while (run)
    {
        if (MW_SerialRecv(&id, &length, data))
        {
            switch ((int)(unsigned char)data[1])
            {
            case ACC:
                acc_x = ((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8);
                acc_y = ((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8);
                acc_z = ((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8);

                imu.linear_acceleration.x = acc_x / 1000.0 * 9.8;
                imu.linear_acceleration.y = acc_y / 1000.0 * 9.8;
                imu.linear_acceleration.z = acc_z / 1000.0 * 9.8;
                break;

            case GYO:
                gyo_x = ((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8);
                gyo_y = ((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8);
                gyo_z = ((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8);

                imu.angular_velocity.x = gyo_x / 10.0 * 0.01745;
                imu.angular_velocity.y = gyo_y / 10.0 * 0.01745;
                imu.angular_velocity.z = gyo_z / 10.0 * 0.01745;
                break;

            case DEG:
		deg_x = ((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8);
                deg_y = ((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8);
                deg_z = ((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8);

                float x = deg_x / 100.0;
                float y = deg_y / 100.0;
                float z = deg_z / 100.0;

                imu.orientation.w = (COS(z) * COS(y) * COS(x)) + (SIN(z) * SIN(y) * SIN(x));
                imu.orientation.x = (COS(z) * COS(y) * SIN(x)) - (SIN(z) * SIN(y) * COS(x));
                imu.orientation.y = (COS(z) * SIN(y) * COS(x)) + (SIN(z) * COS(y) * SIN(x));
                imu.orientation.z = (SIN(z) * COS(y) * COS(x)) - (COS(z) * SIN(y) * SIN(x));
                break;
            }
        }
    }
}
    int main(int argc, char **argv)
    {
        pthread_t thread;

        ros::init(argc, argv, "mw_ahrs");
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("imu", 10);

        MW_SerialOpen("/dev/AHRS", 115200);
        pthread_create(&thread, NULL, AHRS_thread, NULL);

        imu.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
        imu.angular_velocity_covariance = {0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02};
        imu.linear_acceleration_covariance = {0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04};

        imu.linear_acceleration.x = 0;
        imu.linear_acceleration.y = 0;
        imu.linear_acceleration.z = 0;

        imu.angular_velocity.x = 0;
        imu.angular_velocity.y = 0;
        imu.angular_velocity.z = 0;

        imu.orientation.w = 0;
        imu.orientation.x = 0;
        imu.orientation.y = 0;
        imu.orientation.z = 0;

        ros::Rate rate(10);

        while (ros::ok())
        {
            imu.header.frame_id = "imu_link";
            imu.header.stamp = ros::Time::now();

            chatter_pub.publish(imu);

            ros::spinOnce();
            rate.sleep();
        }
        run = false;
        pthread_join(thread,NULL);
        return 0;
    }
 
