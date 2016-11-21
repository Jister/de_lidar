#include "tf20.h"

TF20::TF20(ros::NodeHandle n_private)
{
	//serial
	read_addr = 0;
	write_addr = 0;

	read_valid = false;
	crc_valid = false;
	setup = false;

	//lidar setup
	char Setup_enter[8] = {0xAA, 0x55, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x02};
	memcpy(setup_enter, Setup_enter, sizeof(Setup_enter));
	char Setup_exit[8] = {0xAA, 0x55, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x02};
	memcpy(setup_exit, Setup_exit, sizeof(Setup_exit));
	char Mode_1x1[8] = {0xAA, 0x55, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x08};
	memcpy(mode_1x1, Mode_1x1, sizeof(Mode_1x1));
	char Mode_8x8[8] = {0xAA, 0x55, 0xF0, 0x00, 0x02, 0x00, 0x00, 0x08};
	memcpy(mode_8x8, Mode_8x8, sizeof(Mode_8x8));
	char Time_setup[8] = {0xAA, 0x55, 0xF0, 0x00, 0x14, 0x00, 0x00, 0x40};    //0x14 low , 0x00 high
	memcpy(time_setup, Time_setup, sizeof(Time_setup));

	get_param(n_private);
	serial_init();
	setup = lidar_setup();
	if(setup)
	{
		ROS_INFO("Lidar mode: Output 1*1 data.");
		ROS_INFO("Frequency: %dHz.",frequency);
	}

}

TF20::~TF20()
{
	close(serial_fd);
}

int TF20::set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
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

void TF20::serial_init()
{
	//serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY);
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

bool TF20::lidar_setup()
{
	ret = write(serial_fd, setup_enter, 8);
	if (ret <= 0)  
	{
		ROS_INFO("write Error!");  
		return false;
	}
	memset(read_buf, 0, BUF_SIZE);
	ret = read(serial_fd, read_buf, 8);
	if(ret <= 0)
	{
		ROS_INFO("Setup Error!");  
		return false;
	}else
	{
		ret = write(serial_fd, time_setup, 8);
		if (ret <= 0)    
		{
			ROS_INFO("write Error!");  
			return false;
		}
		memset(read_buf, 0, BUF_SIZE);
		ret = read(serial_fd, read_buf, 8);
		if(ret <= 0)
		{
			ROS_INFO("Setup Error!");  
			return false;
		}else
		{
			ret = write(serial_fd, setup_exit, 8);
			if (ret <= 0)  
			{
				ROS_INFO("write Error!");  
				return false;
			}else
			{
				memset(read_buf, 0, BUF_SIZE);
				ret = read(serial_fd, read_buf, 8);
				if(ret <= 0)
				{
					ROS_INFO("Setup Error!");  
					return false;
				}else
				{
					return true;
				}
			}
		}
	}
}

int TF20::next_data_handle(int addr) {
	return (addr + 1) == MAXSIZE ? 0 : (addr + 1);
}

int TF20::next_data_handle(int addr , int count)     
{     
	int a;
	a = addr;
	for(int i = 0; i < count ; i++)
	{ 
			a = ( (a + 1)  == MAXSIZE ?  0 : ( a + 1 ) ) ;   
	}
	return a;  
}

void TF20::write_data(char data)
{
	*(ring_buf+write_addr) = data;
	write_addr = next_data_handle(write_addr);
}

void TF20::read_data()
{
	memset(read_buf, 0, BUF_SIZE);
	ret = read(serial_fd, read_buf, BUF_SIZE);
	if( ret <= 0 )
	{
		ROS_INFO("read err: %d\n", ret);

	}else
	{
		for(int i = 0 ; i < BUF_SIZE ; i++)
		{
			write_data(read_buf[i]);
		}
	}

	for(int i = 0 ; i < MAXSIZE ; i++)
	{
		if((ring_buf[read_addr] == 0x44) && (ring_buf[next_data_handle(read_addr)] == 0x45) 
			&& (ring_buf[next_data_handle(read_addr,2)] == 0x32) && (ring_buf[next_data_handle(read_addr,3)] == 0x30))  
		{
			for(int j = 0 ; j < 24 ; j++)
			{
				data_buf[j] = ring_buf[read_addr] ;
				read_addr = next_data_handle(read_addr) ;
			}
			read_valid = true;
			break;
		}else
		{
			read_addr = next_data_handle(read_addr) ;
		}
	}

	if(read_valid)
	{
		distance = ((long)data_buf[11]<<24 | (long)data_buf[10]<<16 | (long)data_buf[9]<<8 | (long)data_buf[8]) ;
		amplitude = ((long)data_buf[15]<<24 | (long)data_buf[14]<<16 | (long)data_buf[13]<<8 | (long)data_buf[12]) ;
	}

	crc_data = crc32gen((unsigned int*)data_buf, 5);
	crc_recieved = ((unsigned int)data_buf[23]<<24) | ((unsigned int)data_buf[22]<<16) | ((unsigned int)data_buf[21]<<8) | ((unsigned int)data_buf[20]) ;

	if(crc_data == crc_recieved)
	{
		crc_valid = true;
	}

	if(read_valid && crc_valid)
	//if(read_valid)
	{
		data_valid = true;
	}
}

unsigned int TF20::crc32gen(unsigned int data[], unsigned int size)
{
	unsigned int temp, crc = 0xFFFFFFFF;
	for(int i = 0; i < size; i++)
	{
		temp = data[i];
		for(int j = 0; j < 32; j++)
		{
			if((crc ^ temp) & 0x80000000)
			{
				crc = 0x04C11DB7 ^ (crc << 1);
			}else
			{
				crc <<= 1;
			}

			temp <<= 1;
		}
	}
	return crc;
}

void TF20::get_param(ros::NodeHandle n_private)
{
	if (n_private.getParam("device", device))
	{
		ROS_INFO("Device set to: %s", device.c_str());
	}
	else
	{
		device = "/dev/ttyUSB0";  
		ROS_INFO("Using the default USB device: /dev/ttyUSB0");
	}

	if (n_private.getParam("frequency", frequency))
	{
		unsigned short update_time_ms = 1000 / frequency;
		time_setup[4] = update_time_ms & 0x00ff;      
		time_setup[5] = update_time_ms >> 8;       
		ROS_INFO("Frequency set to: %d", frequency);
	}
	else
	{
		frequency = 50;
		ROS_INFO("Using the default Frequency: 50Hz");
	}
}
