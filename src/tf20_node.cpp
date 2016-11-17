#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define BAUDRATE 	460800
#define DEVICE 	"/dev/ttyUSB0"  
#define MAXSIZE 50

int ret;
int serial_fd;
int direction;
double velocity;
char read_buf[24];
char ring_buf[MAXSIZE];
int read_addr = 0;
int write_addr = 0;

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) ;
void serial_init();
int read();
int next_data_handle(int addr);
void write_data(char data);//put the data into the ringbuffer;
void read_data();//read the data from ringbuffer to the cmd and velocity;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf20_node");
	ros::NodeHandle n;
    ros::Publisher lidar_pub = n.advertise<std_msgs::Float32>("/delidar", 10);
	ros::Rate loop_rate(50);

	serial_init();
	while(ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}
	close(serial_fd);
	return 0;
}


int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
	struct termios newtio,oldtio;  
	if( tcgetattr( fd,&oldtio)  !=  0) 
	{   
		ROS_INFO("SetupSerial 1");  
		return -1;  
	}  
	bzero( &newtio, sizeof( newtio ) );  
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  

	switch( nBits )  
	{  
		case 7:  
			newtio.c_cflag |= CS7;  
			break;  
		case 8:  
			newtio.c_cflag |= CS8;  
			break;  
	}

	switch( nEvent )  
	{  
		case 'O':  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag |= PARODD;  
			newtio.c_iflag |= (INPCK | ISTRIP);  
			break;  
		case 'E':   
			newtio.c_iflag |= (INPCK | ISTRIP);  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag &= ~PARODD;  
	    	break;
		case 'N':    																									
			newtio.c_cflag &= ~PARENB;  
			break;  
	}  

	switch( nSpeed )  
	{  
		case 2400:  
			cfsetispeed(&newtio, B2400);  
			cfsetospeed(&newtio, B2400);  
			break;  
		case 4800:  
			cfsetispeed(&newtio, B4800);  
			cfsetospeed(&newtio, B4800);  
			break;
		case 9600:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
		case 19200:  
			cfsetispeed(&newtio, B19200);  
			cfsetospeed(&newtio, B19200);  
			break; 
		case 57600:  
			cfsetispeed(&newtio, B57600);  
			cfsetospeed(&newtio, B57600);  
			break; 
		case 115200:  
			cfsetispeed(&newtio, B115200);  
			cfsetospeed(&newtio, B115200);  
			break;  
		case 460800:  
			cfsetispeed(&newtio, B460800);  
			cfsetospeed(&newtio, B460800);  
			break; 
		case 921600:  
			cfsetispeed(&newtio, B921600);  
			cfsetospeed(&newtio, B921600);  
			break;   
		default:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
	}  

	if( nStop == 1 )  
	{
		newtio.c_cflag &=  ~CSTOPB;  
	}
	else if ( nStop == 2 )
	{  
		newtio.c_cflag |=  CSTOPB;  
	} 

	newtio.c_cc[VTIME]  = 10;
	newtio.c_cc[VMIN] = 24;
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
		ROS_INFO("com set error!");  
		return -1;  
	}  
	return 0;  
} 

void serial_init()
{
	//serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	serial_fd = open(DEVICE, O_RDWR | O_NOCTTY);
	if (serial_fd == -1)
	{
		ROS_INFO("Open Error!");  
		exit(1);    
	}  
	ret = set_serial(serial_fd, BAUDRATE, 8, 'N', 1);
	if (ret == -1)  
	{
		ROS_INFO("Set Serial Error!");  
		exit(1);  
	}
}

int next_data_handle(int addr) {
	return (addr + 1) == MAXSIZE ? 0 : (addr + 1);
}

void write_data(char data)
{
	*(ring_buf+write_addr) = data;
	write_addr = next_data_handle(write_addr);
}

void read_data()
{
	if (ring_buf[read_addr] == 'M')
	{
		direction = ring_buf[read_addr + 1]-48;
        velocity = (ring_buf[read_addr + 4] - 48) / 300.0;
		for (int i = 0; i < 6; i++) {
			read_addr = next_data_handle(read_addr);
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			read_addr = next_data_handle(read_addr);
			if (ring_buf[read_addr] == 'M')
			{
				break;
			}
		}
	}
}

int read()
{
	int ret;

	memset(read_buf, 0, 50);
	ret = read(serial_fd, read_buf, 6);
	if (ret < 0)
	{
		ROS_INFO("read error!");
		return -1;
	}
	else if (ret == 0)
	{
		//ROS_INFO("No data!");
		return 0;
	}
	else{
        /*ROS_INFO("data:%c",read_buf[0]);
		ROS_INFO("data:%c",read_buf[1]);
		ROS_INFO("data:%c",read_buf[2]);
		ROS_INFO("data:%c",read_buf[3]);
		ROS_INFO("data:%c",read_buf[4]);
        ROS_INFO("data:%c",read_buf[5]);
        return ret;*/
	}
}