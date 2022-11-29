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

// R: [x-axis 90 degrees -> z-axis 90 degrees]
// T: [x, y, z] = [] (measured)
void frameTransfer(const geometry_msgs::Point::ConstPtr& point) {
	pos << point -> x, point -> y, point -> z;
	FrameTransfer(pos);

	Eigen::Matrix3d R1, R2;
        Eigen::Vector3d T(0.135, 0.485, 0.360);
        float theta_x = 90*M_PI/180;
        float theta_z = 90*M_PI/180;
        R1 << 1,             0,            0,
              0,  cos(theta_x), sin(theta_x),
              0, -sin(theta_x), cos(theta_x);
        R2 << cos(theta_z), sin(theta_z), 0,
             -sin(theta_z), cos(theta_z), 0,
                         0,            0, 1;
        trans_pos = R2*R1*pos+T;

	position.linear.x = trans_pos.x();
	position.linear.y = trans_pos.y();
	position.linear.z = trans_pos.z();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "camera_extrinsics");
	ros::NodeHandle nh;
	ros::Subscriber sub_cam2disk_pos;
	ros::Publisher pub_world2disk_pos;
	
	sub_cam2disk_pos = nh.subscribe("camera/disk_position", 30, frameTransfer);
	ros::Rate r(100);

	while(ros::ok())
	{
		pub_world2disk_pos = nh.advertise<geometry_msgs::Twist>("world/disk_position", 100);
		pub_world2disk_pos.publish(position);

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
