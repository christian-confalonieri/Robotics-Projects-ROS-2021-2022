#include <cstdio>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "project2/PrintTrajectory.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
using namespace std;

typedef struct pose {

    double x;
    double y;

}Pose;

typedef struct poses {

    std::vector<Pose> poses;

}Poses;

class pub_sub
{

std_msgs::String messagio;
std_msgs::String messagio2;

private:
ros::NodeHandle n; 

ros::Subscriber sub;
ros::Subscriber sub2;
ros::Publisher pub; 
ros::Timer timer1;
ros::ServiceServer print_trajectory_service;
	
	
public:
	nav_msgs::Path path;
	Poses poses;

  	pub_sub(){
  	sub = n.subscribe("odom", 1, &pub_sub::odomCallback, this);
	sub2 = n.subscribe("amcl_pose",1, &pub_sub::poseAMCLCallback, this); 
	pub = n.advertise<nav_msgs::Path>("path",10);  
	print_trajectory_service = n.advertiseService("print_trajectory", &pub_sub::print_trajectory_service_function, this);
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	geometry_msgs::PoseStamped pose;

	path.header = msg->header;
	pose.header = msg->header;
	pose.pose = msg->pose.pose;
	path.poses.push_back(pose);
	pub.publish(path);

	Pose local_pose;

	local_pose.x = msg->pose.pose.position.x;
	local_pose.y = msg->pose.pose.position.y;
	poses.poses.push_back(local_pose);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	tf::Transform transform;

  	transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
  	transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
  	static tf::TransformBroadcaster br;
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

bool print_trajectory_service_function(project2::PrintTrajectory::Request &req, project2::PrintTrajectory::Response &res) {
	std::string packagePath = ros::package::getPath("project2") + "/maps/";

    Mat image = imread(
        packagePath + "map.pgm",
        IMREAD_COLOR);
  
    if (!image.data) {
        std::cout << "Could not open or "
                     "find the image";
        return false;
    }
	
	for(int i=1; i<poses.poses.size(); i++) {

		int origin = 400;
		int zoom_x = 20;
		int zoom_y = -20;

		Point p1((poses.poses[i-1].x * zoom_x) + origin, (poses.poses[i-1].y * zoom_y) + origin); 
		Point p2((poses.poses[i].x * zoom_x) + origin, (poses.poses[i].y * zoom_y) + origin);

    	int thickness = 2;
  
    	line(image, p1, p2, Scalar(0, 255, 0), thickness, LINE_8);
	}
	
	remove((packagePath + "map_trajectory.png").c_str());
	imwrite(packagePath + "map_trajectory.png", image);

	res.response = "The image containing the map and trajectory of the robot was created in the folder: \"maps\"";
	return true;
}

};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "subscribe_and_publish");
 	pub_sub my_pub_sub;
 	ros::spin();
 	return 0;
}
