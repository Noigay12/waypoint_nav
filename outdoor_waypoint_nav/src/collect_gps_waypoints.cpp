#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <fstream>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>

bool collect_request = true;
bool continue_collection = true;
double lati_point=0, longi_point=0, lati_last=0, longi_last=0;
double min_coord_change = 10 * pow(10,-6);

void filtered_gps_CB(const sensor_msgs::NavSatFix gps_msg)
{
		lati_point = gps_msg.latitude;
		longi_point = gps_msg.longitude;
}

int main(int argc, char** argv)
{
	// Initialize variables
		int numWaypoints = 0;
		std::string path_local;

    // Initialize node and time
		ros::init(argc, argv, "collect_gps_waypoints"); // Initiate node called collect_gps_waypoints
		ros::NodeHandle n;
		ros::Time::init();
		ros::Time time_last;
		ros::Time time_current;
		ros::Duration duration_min(1);

    // Initiate subscribers
		ros::Subscriber sub_gps = n.subscribe("/outdoor_waypoint_nav/gps/filtered", 100, filtered_gps_CB);
		ROS_INFO("Initiated collect_gps_waypoints node");

	// Initiate publisher to send end of node message
		ros::Publisher pubCollectionNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/collection_status",100);

    // Read file path and create/open file
    	ros::param::get("/outdoor_waypoint_nav/coordinates_file", path_local);
		std::string path_abs =  ros::package::getPath("outdoor_waypoint_nav") + path_local;	
		std::ofstream coordFile (path_abs.c_str());
		ROS_INFO("Saving coordinates to: %s", path_abs.c_str());

	if(coordFile.is_open())
	{
		while(continue_collection)
		{
			ros::spinOnce();
			time_current = ros::Time::now();
			if((collect_request == true) && (time_current - time_last > duration_min))
			{	
				// Check that there was sufficient change in position between points
				// This makes the move_base navigation smoother and stops points from being collected twice
				double difference_lat = abs((lati_point - lati_last)*pow(10,6))*pow(10,-6);
				double difference_long = abs((longi_point - longi_last)*pow(10,6))*pow(10,-6);

				if( (difference_lat > min_coord_change) || (difference_long > min_coord_change))
				{
					// Write waypoint
					ROS_INFO("You have collected another waypoint!");
					std::cout << std::endl;
					numWaypoints++;
					coordFile << std::fixed << std::setprecision(8) << lati_point << " " << longi_point << std::endl;
					lati_last = lati_point;
					longi_last = longi_point;
				}
				time_last = time_current;
			}
			else{}
			ros::spinOnce();
		}
	
		coordFile.close();
		ROS_INFO("End request registered.");
	}
	else
	{
		ROS_ERROR("Unable to open file.");
		ROS_INFO("Exiting..");
	}

	ROS_INFO("Closed waypoint file, you have collected %d waypoints.", numWaypoints);
	ROS_INFO("Ending node...");

	// Notify joy_launch_control that calibration is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubCollectionNodeEnded.publish(node_ended);

	ros::shutdown();
	return 0;
}
