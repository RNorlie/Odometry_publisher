#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"

#include <cmath>
#include <fstream>
#include <iostream>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <bits/stdc++.h>

#define ABS_SCALER 20480
#define TRAC_SCALER 0.58
#define ABS_WRAP 16777215
#define max_pos_index 1
#define max_vel_index 1
#define dt 0.050

int serial_port = open("/dev/ttyUSB0", O_RDWR);

int parse_bytes(char * buffer, int index)
{
    int a = (int)(buffer[index] << 24 | buffer[index+1] << 16 | buffer[index+2] << 8 | buffer[index+3]);
    return a;
}

class EncoderReader
{
public:
    EncoderReader(ros::NodeHandle *n)
    {
        odom_publisher = n->advertise<geometry_msgs::TwistStamped>("robot/velocity", 2);
    }

    void read_timer_callback()
    {
	clock_t start, end;
	start = clock();

        static int pos_array [4];
        static float vel_array [4];

        static int pos_index = 0;
        static int vel_index = 0;

        static int initial_absolute_encoder_pulses = INT_MAX;
        static int absolute_encoder_pulses = 0;
        static int traction_encoder_pulses_forward = 0;
        static int traction_encoder_pulses_backward = 0;

        char read_buf [256];
        int num_bytes_read = read(serial_port, &read_buf, sizeof(read_buf));

        if (num_bytes_read != 12)
           return;
        printf("Read %i bytes. Received message: %s\n", num_bytes_read, read_buf);
        int i = 0;

        absolute_encoder_pulses = parse_bytes(read_buf, i);
        if(initial_absolute_encoder_pulses == INT_MAX)
            initial_absolute_encoder_pulses = absolute_encoder_pulses;
        printf("Abs_enc as an int: %i\n", absolute_encoder_pulses);
        i+=4;

        //convert incremental encoder_a string to int
        traction_encoder_pulses_forward = parse_bytes(read_buf, i);
        printf("inc_enc_a as an int: %i\n", traction_encoder_pulses_forward);
        i+=4;

        //convert incremental encoder_b string to int
        traction_encoder_pulses_backward = parse_bytes(read_buf, i);
        printf("inc_enc_b as an int: %i\n", traction_encoder_pulses_backward);

        float current_pos = ((traction_encoder_pulses_forward - traction_encoder_pulses_backward)*TRAC_SCALER);
        float alpha = (((float)absolute_encoder_pulses - (float)initial_absolute_encoder_pulses)/ABS_SCALER);

        printf("alpha = %.6f\n", alpha);

        //populate pos_array

        if(pos_index <= max_pos_index) {
            pos_array[pos_index] = current_pos;
            pos_index++;
        }
        else
        {
            for(int i = 0; i < max_pos_index; i++)
            {
                pos_array[i] = pos_array[i+1];
            }
            pos_array[max_pos_index] = current_pos;
        }

        //populate vel_array

        if(pos_index > max_pos_index) {
            float current_vel = ((float)pos_array[max_pos_index] - (float)pos_array[0]) / dt;
            if(vel_index <= max_vel_index) {
                vel_array[vel_index] = current_vel;
                vel_index++;
            }
            else
            {
                float sum = 0;
                for(int i = 0; i < max_vel_index; i++)
                {
                    vel_array[i] = vel_array[i+1];
                }
                vel_array[max_vel_index] = current_vel;

                for(int i = 0; i <= max_vel_index; i++)
                {
                    sum += vel_array[i];
                }
                float average_velocity = sum / (float)max_vel_index+1;
		printf("average velocity = %.6f\n", average_velocity);
            }
        }
	//end = clock();
	//double time_taken = double(end - start) / double(1500000000);
	//std::cout << time_taken << std::endl;
    }

    void publish_timer_callback()
    {
        static int count = 0;
        geometry_msgs::TwistStamped twist_stamped_msg;

        twist_stamped_msg.header.seq = count;
        twist_stamped_msg.header.stamp = ros::Time::now();
        //twist_stamped_msg.header.frame_id =

        twist_stamped_msg.twist.linear.x = 0;
        twist_stamped_msg.twist.linear.y = 0;
        twist_stamped_msg.twist.linear.z = 0;

        twist_stamped_msg.twist.angular.x = 0;
        twist_stamped_msg.twist.angular.y = 0;
        twist_stamped_msg.twist.angular.z = 0;

        odom_publisher.publish(twist_stamped_msg);
        count++;

        printf("publishing!\n");
    }

private:
    ros::Publisher odom_publisher;
};



int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    EncoderReader encoderReader(&n);

    ros::Timer read_timer = n.createTimer(ros::Duration(0.1/40.0), std::bind(&EncoderReader::read_timer_callback, encoderReader));
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.1/20.0), std::bind(&EncoderReader::publish_timer_callback, encoderReader));

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    //configuring the UART

    tty.c_cflag &= ~PARENB; //No parity
    tty.c_cflag &= ~CSTOPB; //1 stopbit
    tty.c_cflag &= ~CSIZE;  //Clear size bits
    tty.c_cflag |= CS8;     //8 data bits
    tty.c_cflag &= ~CRTSCTS;//disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; //enable reading

    tty.c_lflag &= ~ICANON; //disable canonical mode
    tty.c_lflag &= ~ECHO;   //disbale echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 12;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    //flush input buffer
    sleep(1);
    tcflush(serial_port, TCIFLUSH);

    ros::spin();
}

/*

    //x linear velocity in meters/?
    //float linear_vel = 1000*(speed*(cos(alpha)));

    //angular velocity in meters/?
    //float angular_vel = 1000*((speed/d)*(sin(alpha)));

*/

//    int bytes_in_buf;
//    ioctl(serial_port, FIONREAD, &bytes_in_buf);

//    printf("There are %i bytes in the input buffer\n", bytes_in_buf);
