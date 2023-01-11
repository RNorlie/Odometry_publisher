/*
** ###################################################################
**     Filename  : publisher.cpp
**     Processor : Raspberry Pi4B
**
**     © 2020 Mitsubishi Logisnext Americas Inc.
**
**     Mitsubishi Logisnext Americas Group – Houston Campus
**     2121 W Sam Houston Parkway North
**     Houston, TX 77043
**     United States
**
**     Mitsubishi Logisnext Americas Group – Marengo Campus
**     240 N Prospect St
**     Marengo, IL 60152
**     United States
**
** ###################################################################
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt32.h"
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

//#define ABS_WRAP 0.017578125
#define ABS_SCALER 0.01758
#define ABSOLUTE_ZERO 16743664.0
#define TRAC_SCALER 0.00058
#define FRAME_LENGTH 1.252
#define DEG_TO_RAD 0.01745
#define ROT_TO_RAD 6.28319
#define MAX_SPEED 1.5 // m/s

#define max_index 3
#define publish_freq 25.0
#define read_freq 100.0

#define MAX_STR_LEN 10

int serial_port = open("/dev/ttyUSB0", O_RDWR);

//Global variables to publish
double linear_vel_x = 0.0;
double linear_vel_y = 0.0;
double angular_vel_z = 0.0;

bool rec_pending = false;

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

double sum_array(double *array, int size)
{
    double sum = 0;
    for(unsigned int i = 0; i < size+1; i++)
    {
        sum += array[i];
    }
    return sum;
}

uint32_t parse_bytes(char *buffer, int index)
{
    uint32_t byte = (uint32_t)(buffer[index] << 24 | buffer[index+1] << 16 | buffer[index+2] << 8 | buffer[index+3]);
    return byte;
}

uint8_t get_fmi_code(char *buffer, int index)
{
    uint8_t code = (uint8_t)(buffer[index]);
    return code;
}


void transmit_watchdog_callback(const ros::TimerEvent& event)
{
    if (rec_pending) {
        printf("Timeout waiting for data...\n");
        rec_pending = false;
    }
}


class EncoderReader
{
public:
    EncoderReader(ros::NodeHandle *n)
    {
        odom_publisher = n->advertise<geometry_msgs::TwistStamped>("robot/velocity", 1);
        absolute_publisher = n->advertise<std_msgs::UInt32>("encoder/absolute", 1);
        forward_publisher = n->advertise<std_msgs::UInt32>("encoder/forward", 1);
        backward_publisher = n->advertise<std_msgs::UInt32>("encoder/backward", 1);
    }

    void publish_timer_callback()
    {
        static int count = 0;
        geometry_msgs::TwistStamped twist_stamped_msg;

        twist_stamped_msg.header.seq = count;
        twist_stamped_msg.header.stamp = ros::Time::now();

        twist_stamped_msg.twist.linear.x = linear_vel_x;
        twist_stamped_msg.twist.linear.y = linear_vel_y;

        twist_stamped_msg.twist.angular.z = angular_vel_z;

        odom_publisher.publish(twist_stamped_msg);
        count++;

        double delta_time = (ros::Time::now() - later).toSec();
        later = ros::Time::now();

        printf("Time since last publish: %f seconds\n", delta_time);
    }

    void pollForData()
    {
        unsigned char read_buf[80];
        char sendBuff[MAX_STR_LEN];
        memset(&sendBuff[0], 0, MAX_STR_LEN);
        snprintf(&sendBuff[0], MAX_STR_LEN, "cts");
        int num_bytes_read = 0;
        int fmi_code;
        uint32_t data_value;

        while(ros::ok())
        {
            // Send request for data
            //if (!rec_pending) {
            //    write(serial_port, sendBuff, sizeof(sendBuff));
            //    printf("Wrote %s\n", sendBuff);
            //    rec_pending = true;
            //}

            //ros::Time start_read = ros::Time::now();
            //memset(read_buf, 0, sizeof(read_buf));
            num_bytes_read = 0;
            num_bytes_read = read(serial_port, &read_buf, sizeof(read_buf));

            printf("Read %i bytes. \n", num_bytes_read);

            unsigned char serial_index = 0;
            printf("Raw Data: ");

            while (serial_index < (num_bytes_read)) {

                /*
                if ((serial_index == 0) || (serial_index == 5) || (serial_index == 10)) {
                    fmi_code = get_fmi_code(read_buf, serial_index);
                    printf("FMI=%u ", fmi_code);
                    serial_index += 1;
                } 
                if ((serial_index == 1) || (serial_index == 6) || (serial_index == 11)) {
                    data_value = parse_bytes(read_buf, serial_index);
                    printf("Data=%lu ", (unsigned long)data_value);
                    serial_index += 4;
                } 
                */
                printf("0x%x ", read_buf[serial_index]);
                //printf("%x", read_buf[serial_index]);           

                serial_index += 1;
            }
            printf("\n");


            /*

            traction_encoder_forward_fmi_code = get_fmi_code(read_buf, serial_index);
            printf("incremental encoder backward fmi code: %d\n", traction_encoder_forward_fmi_code);
            serial_index+=1;
            traction_encoder_pulses_forward = parse_bytes(read_buf, serial_index);
            printf("inc_enc_b as an int: %i\n\n", traction_encoder_pulses_forward);
            serial_index+=4;

            traction_encoder_backward_fmi_code = get_fmi_code(read_buf, serial_index);
            printf("incremental encoder backward fmi code: %d\n", traction_encoder_backward_fmi_code);
            serial_index+=1;
            traction_encoder_pulses_backward = parse_bytes(read_buf, serial_index);
            printf("inc_enc_a as an int: %i\n", traction_encoder_pulses_backward);
            serial_index+=4;

            absolute_fmi_code = get_fmi_code(read_buf, serial_index);
            printf("absolute encoder fmi code: %d\n", absolute_fmi_code);
            serial_index+=1;
            absolute_encoder_pulses = parse_bytes(read_buf, serial_index);
            printf("Abs_enc as an int: %i\n", absolute_encoder_pulses);
            */

            //printf("x vel = %f\n", linear_vel_x);
            //printf("y vel = %f\n", linear_vel_y);
            //printf("z vel = %f\n", angular_vel_z);

            //ros::Time end_read = ros::Time::now();
            //ros::Duration time_elapsed = end_read - start_read;
            //printf("time elapsed = %f seconds\n", time_elapsed.toSec());
            ros::spinOnce(); // if publish_timer's callback has been called, process it now.
        }
    }


private:

    double dt;

    ros::Publisher odom_publisher, absolute_publisher, forward_publisher, backward_publisher;
    ros::Time start_time = ros::Time::now();
    ros::Time then = start_time;
    ros::Time later = start_time;

    double displacement_array [max_index+1];
    double linear_x_vel_array [max_index+1];
    double linear_y_vel_array [max_index+1];
    double angular_vel_array [max_index+1];
    double dt_array [max_index+1];

    int absolute_encoder_pulses = 0;
    int traction_encoder_pulses_forward = 0;
    int traction_encoder_pulses_backward = 0;

    int absolute_fmi_code = 0;
    int traction_encoder_forward_fmi_code = 0;
    int traction_encoder_backward_fmi_code = 0;

    int invalid_readings = 0;
    int dropped_readings = 0;
    int failure_mode_identifier = 0;

    double theta = 0.0;
    double theta_rad = 0.0;
    double wheel_velocity = 0.0;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;

    EncoderReader encoderReader(&n);

    //ros::Timer read_timer = n.createTimer(ros::Duration(1.0/read_freq), std::bind(&EncoderReader::read_timer_callback, encoderReader));
    //ros::Timer publish_timer = n.createTimer(ros::Duration(1.0/publish_freq), std::bind(&EncoderReader::publish_timer_callback, encoderReader));
    ros::Timer transmit_timer = n.createTimer(ros::Duration(0.01), transmit_watchdog_callback);

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    //configuring the UART

    tty.c_cflag &= ~PARENB; // Disable Parity
    //tty.c_cflag &= ~PARODD; // Even Parity
    tty.c_cflag &= ~CSTOPB; //1 stopbit
    tty.c_cflag &= ~CSIZE;  //Clear size bits
    tty.c_cflag |= CS8;     //8 data bits
    tty.c_cflag &= ~CRTSCTS;//disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; //enable reading

    tty.c_lflag &= ~ICANON; //disable canonical mode
    tty.c_lflag &= ~ECHO;  //disbale echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_iflag &= IGNPAR;

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 15; // Read up to 15 bytes

    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    //flush input buffer
    sleep(1);
    tcflush(serial_port, TCIFLUSH);

    encoderReader.pollForData();
}
