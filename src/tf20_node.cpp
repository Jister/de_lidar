#include "tf20.h"
#include "de_lidar/Lidar.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf20_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	ros::Publisher lidar_pub = n.advertise<de_lidar::Lidar>("/lidar", 10);
	TF20 tf20(n_private);
	ros::Rate loop_rate(tf20.frequency);

	bool first_measure = false;
	de_lidar::Lidar msg;
	
	while(ros::ok())
	{
		tf20.read_data();
		// ROS_INFO("crc calculated:%lu",crc_data);
		// ROS_INFO("crc recieved:%lu", crc_recieved);
		if(tf20.data_valid)
		{
			if(!first_measure)
			{
				ROS_INFO("Lidar started!");
				first_measure = true;
			}

			msg.header.stamp = ros::Time::now();
			msg.distance.data = tf20.distance / 100.0;
			msg.amplitude.data = tf20.amplitude;
			lidar_pub.publish(msg);

			tf20.data_valid = false;
			
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

