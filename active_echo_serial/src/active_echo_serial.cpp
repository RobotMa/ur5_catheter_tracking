#include "ros/ros.h"
#include "std_msgs/String.h"

// Include custom message
#include <active_echo_serial/Num.h>

// For clean serial port close
#include <signal.h>

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <iostream>
#include <stdlib.h>  /* atoi */


// For clean serial port close
int fd = 0;

int open_port(std::string& port)
{   
	int fd; /* File descriptor for the port */

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	// fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	fd = open(port.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{     
		/* Could not open the port. */
		return (-1);
	}     
	else
	{
		fcntl(fd, F_SETFL, 0);
	}

	return (fd);
}
bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;

	struct termios config;
	if (!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	if (tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
	config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
	config.c_oflag &= ~ONOEOT;
#endif

	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	// config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_lflag &= ~(ECHO | IEXTEN | ISIG);

	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//

	config.c_cc[VMIN] = 46;  // Qianli: 46 is the number of charaters that will be read every time
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	//tcgetattr(fd, &options);

	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

			// These two non-standard (by the 70'ties ) rates are fully supported on
			// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	//
	// Finally, apply the configuration
	//
	if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

void close_port(int fd)
{
	close(fd);
}

void mySigintHandler(int sig)
{
	close_port( fd );
	ros::shutdown();
}


int main(int argc, char **argv)
{
	// Open the serial port (This should be serial port for the active echo microcontroller)
	std::string portname = "/dev/ttyUSB0";

	ros::init(argc, argv, "active_echo_publisher");

	ros::NodeHandle n;
	ros::Publisher n_pub = n.advertise<active_echo_serial::Num>("active_echo_data", 20);

	// Keep checking the existence of serial port 
	int fd = open_port(portname);

	std::cout << fd << std::endl;

	int res;

	// Set the buad rate, data bits and stop bit
	setup_port(fd, 921600, 8, 1, false, false);
	
	// Lower loop rate will render the size of buf to be 255
	// Higher loop rate is desired/required 
	// Current tested safe threshold is 50 or above
	ros::Rate loop_rate(50);

	// Define the names of the four output integers
	std::string l_all();
	std::string l_ta();
	std::string dly();
	std::string tc();

	while(ros::ok())
	{

		if ( fd!=-1 )
		{
			
			// For clean serial port close	
			signal(SIGINT, mySigintHandler);
			/* 
			   Publish to active_echo_serial/Num Rostopic if serial communication
			   is established
			   */

			char buf[255];
			// Initialize buf
			for (int i = 0; i< 255; i++)
			{
				buf[i] = 0;
			}

			// Get the serial port signal
			// res: number of characters received
			std::cout << "Serial port working before starting to read " << std::endl; 
			res = read(fd, buf, 255);
			std::cout << buf << std::endl;

			// Convert char buf to string full_info
			std::string full_info(buf);

			std::cout << full_info.length() << std::endl;

			// Initialize active echo ROS msg
			active_echo_serial::Num msg;

			// Parse the string into four variables once the serial
			// port is able to get a string with full length (46)
			if (full_info.length()%46 == 0 && full_info.length() > 0)
			{
				// std::cout << full_info.at(4) << std::endl;
				std::string l_all = full_info.substr(6,6);
				std::string  l_ta = full_info.substr(18,6);
				std::string   dly = full_info.substr(29,6);
				std::string    tc = full_info.substr(39,6);
				// std::cout << l_all << std::endl;
				// std::cout <<  l_ta << std::endl;
				// std::cout <<   dly << std::endl;
				// std::cout <<    tc << std::endl;

				std::string::size_type sz; // alias of size_t

				// Convert string numbers to integers
				int l_all_v = atoi(l_all.c_str());
				int  l_ta_v = atoi( l_ta.c_str());
				int   dly_v = atoi(  dly.c_str());
				int    tc_v = atoi(   tc.c_str());

				msg.l_all = l_all_v;
				msg.l_ta  = l_ta_v;
				msg.dly   = dly_v;
				msg.tc    = tc_v;
				// std::cout << l_all_v << " " << dly_v <<std::endl;
				
				// Only publish when tc_v is not equal to zeros
				// This is condition of successfull point detection
				if (tc_v == 0){ ROS_INFO("Can't detect point"); }
				else{ n_pub.publish(msg); }

			}

			
		}
		else { std::cout << "No active echo serial port detected." << std::endl; }
		loop_rate.sleep();
	}

	return 0;
}
