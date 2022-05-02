/* Camera frame to the base frame of kinova-jaco */

#include "ros/ros.h"
#include "geometry_msgs/Point.h" 
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

geometry_msgs::Twist position; // store object's coordinate
Eigen::Vector3d pos;
Eigen::Vector3d trans_pos;

// R: [x-axis 90 degrees -> z-axis 180 degrees]
// T: [x, y, z] = [0.110, 0.523, 0.393] (measured)
void FrameTransfer(Eigen::Vector3d pos)
{
	Eigen::Matrix3d R1, R2;
	Eigen::Vector3d T(0.135, 0.485, 0.360); // try for eliminating error
	float theta_x = 90*M_PI/180;
	float theta_z = 180*M_PI/180;
	R1 << 1,             0,            0,
	      0,  cos(theta_x), sin(theta_x),
	      0, -sin(theta_x), cos(theta_x);
	R2 << cos(theta_z), sin(theta_z), 0,
	     -sin(theta_z), cos(theta_z), 0,
			 0,	       0, 1;
	trans_pos = R2*R1*pos+T;
}

void Transfer(const geometry_msgs::Point::ConstPtr& point)
{
	pos << point -> x, point -> y, point -> z;
	//std::cout << "Camera Frame" << std::endl;
	//ROS_INFO("x: %f, y: %f, z: %f", pos.x(), pos.y(), pos.z());
	FrameTransfer(pos);

	position.linear.x = trans_pos.x();
	position.linear.y = trans_pos.y();
	position.linear.z = trans_pos.z();

	//std::cout << "Kinova Frame" << std::endl;
	//ROS_INFO("x: %f, y: %f, z: %f", position.x, position.y, position.z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_tf");
	ros::NodeHandle nh;
	ros::Subscriber sub_cam_coordinate;
	ros::Publisher pub_object_position;
	
	sub_cam_coordinate = nh.subscribe("camera_coordinate", 100, Transfer);
	ros::Rate r(100);

	while(ros::ok())
	{
		pub_object_position = nh.advertise<geometry_msgs::Twist>("tennis_position", 100);
		pub_object_position.publish(position);

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
