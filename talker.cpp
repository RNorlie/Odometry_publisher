#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <array>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <bits/stdc++.h>

#define ABS_SCALER 20480
#define TRAC_SCALER 0.00058
#define ABS_WRAP 16777215
#define FRAME_LENGTH 10

#define max_index 1
#define publish_freq 20.0
#define read_freq 40.0

int serial_port = open("/dev/ttyUSB0", O_RDWR);

double linear_vel_x = 0.0;
double linear_vel_y = 0.0;
double angular_vel_z = 0.0;

void print_array(double *array, int size)
{
    for(unsigned int i = 0; i < size+1; i++)
    {
	printf("array[%d] = %f\n", i, array[i]);
    }
}

void shift_array(double *array, int size)
{
    for(unsigned int i = 0; i < size; i++)
    {
	array[i] = array[i+1];
    }
}

double average_array(double *array, int size)
{
    double sum = 0;
    for(unsigned int i = 0; i < size+1; i++)
    {
	sum += array[i];
    }
    return (sum / (double)(size+1));
}

int parse_bytes(char *buffer, int index)
{
    int a = (int)(buffer[index] << 24 | buffer[index+1] << 16 | buffer[index+2] << 8 | buffer[index+3]);
    return a;
}

class EncoderReader
{
public:
    EncoderReader(ros::NodeHandle *n)
    {
        odom_publisher = n->advertise<geometry_msgs::TwistStamped>("robot/velocity", 1);
    }

    void read_timer_callback()
    {
	//ros::Time start_read = ros::Time::now();

        char read_buf [256];
        int num_bytes_read = read(serial_port, &read_buf, sizeof(read_buf));

        if (num_bytes_read == 12) { //if invalid reading, do not update encoder values

            //printf("Read %i bytes. Received message: %s\n", num_bytes_read, read_buf);
            unsigned int serial_index = 0;

            absolute_encoder_pulses = parse_bytes(read_buf, serial_index);
            if(initial_absolute_encoder_pulses == INT_MAX)
                initial_absolute_encoder_pulses = absolute_encoder_pulses;
            printf("Abs_enc as an int: %i\n", absolute_encoder_pulses);
            serial_index+=4;

            traction_encoder_pulses_backward = parse_bytes(read_buf, serial_index);
            printf("inc_enc_a as an int: %i\n", traction_encoder_pulses_backward);
            serial_index+=4;

            traction_encoder_pulses_forward = parse_bytes(read_buf, serial_index);
            printf("inc_enc_b as an int: %i\n\n", traction_encoder_pulses_forward);

        }

        double current_pos = (((double)traction_encoder_pulses_forward - (double)traction_encoder_pulses_backward)*TRAC_SCALER); //distance the drive wheel has travelled
        double alpha = (((double)absolute_encoder_pulses - (double)initial_absolute_encoder_pulses)/ABS_SCALER); //angle of drive wheel

	//printf("alpha = %.6f\n", alpha);
	//printf("current_pos = %f\n", current_pos);

        //populate pos_array

        if(pos_index < max_index+1) {
            pos_array[pos_index] = current_pos;
            pos_index++;
	    return;
        }

	shift_array(pos_array, max_index);
	pos_array[max_index] = current_pos;

        double dt = (ros::Time::now() - then).toSec(); //time passed since last velocity calculation
	then = ros::Time::now();

	//printf("time elapsed = %f seconds\n", dt);

	//populate velocity arrays

	if(pos_index > max_index) {
            double wheel_velocity = ((double)pos_array[max_index] - (double)pos_array[0]) / dt;

	    //printf("wheel_velocity = %f\n", wheel_velocity);

	    double current_x_velocity = wheel_velocity * cos(alpha + theta);
	    double current_y_velocity = wheel_velocity * sin(alpha + theta);
	    double current_angular_velocity = (wheel_velocity * sin(alpha)) / FRAME_LENGTH;

	    //printf("theta = %f\n", theta);

	    theta += current_angular_velocity * dt; //angle of truck heading

	    //average linear x velocity

            if(linear_x_index < max_index+1) {
                linear_x_vel_array[linear_x_index] = current_x_velocity;
                linear_x_index++;
		return;
            }

	    shift_array(linear_x_vel_array, max_index);
            linear_x_vel_array[max_index] = current_x_velocity;

            linear_vel_x = average_array(linear_x_vel_array, max_index);

	    //average linear y velocity

            if(linear_y_index < max_index+1) {
                linear_y_vel_array[linear_y_index] = current_y_velocity;
                linear_y_index++;
		return;
            }

            shift_array(linear_y_vel_array, max_index);
            linear_y_vel_array[max_index] = current_y_velocity;

            linear_vel_y = average_array(linear_y_vel_array, max_index);

	    //average angular z velocity

            if(angular_index < max_index+1) {
                angular_vel_array[angular_index] = current_angular_velocity;
                angular_index++;
		return;
            }

            shift_array(angular_vel_array, max_index);
            angular_vel_array[max_index] = current_angular_velocity;

            angular_vel_z = average_array(angular_vel_array, max_index);

	    printf("x vel = %f\n", linear_vel_x);
            printf("y vel = %f\n", linear_vel_y);
            printf("z vel = %f\n", angular_vel_z);

        }
	//ros::Time end_read = ros::Time::now();
	//ros::Duration time_elapsed = end_read - start_read;
	//printf("time elapsed = %f seconds\n", time_elapsed.toSec());
    }

    void publish_timer_callback()
    {

        static int count = 0;
        geometry_msgs::TwistStamped twist_stamped_msg;

        twist_stamped_msg.header.seq = count;
        twist_stamped_msg.header.stamp = ros::Time::now();
        //twist_stamped_msg.header.frame_id =

        twist_stamped_msg.twist.linear.x = linear_vel_x;
        twist_stamped_msg.twist.linear.y = linear_vel_y;
        twist_stamped_msg.twist.linear.z = 0;

        twist_stamped_msg.twist.angular.x = 0;
        twist_stamped_msg.twist.angular.y = 0;
        twist_stamped_msg.twist.angular.z = angular_vel_z;

        odom_publisher.publish(twist_stamped_msg);
        count++;

        //printf("publishing!\n");
    }

private:
    double dt;

    ros::Publisher odom_publisher;
    ros::Time start_time = ros::Time::now();
    ros::Time then = start_time;

    double pos_array [max_index+1];
    double linear_x_vel_array [max_index+1];
    double linear_y_vel_array [max_index+1];
    double angular_vel_array [max_index+1];

    int pos_index = 0;
    int linear_x_index = 0;
    int linear_y_index = 0;
    int angular_index = 0;

    int initial_absolute_encoder_pulses = INT_MAX;
    int absolute_encoder_pulses = 0;
    int traction_encoder_pulses_forward = 0;
    int traction_encoder_pulses_backward = 0;

    double theta = 0.0;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    EncoderReader encoderReader(&n);

    ros::Timer read_timer = n.createTimer(ros::Duration(0.1/read_freq), std::bind(&EncoderReader::read_timer_callback, encoderReader));
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.1/publish_freq), std::bind(&EncoderReader::publish_timer_callback, encoderReader));

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
    tty.c_cc[VMIN] = 0;

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
