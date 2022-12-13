#include "ros/ros.h"
#include "geometry_msgs/Point.h" 
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

geometry_msgs::Point position; // store object's coordinate
Eigen::Vector3d pos;
Eigen::Vector3d trans_pos;
Eigen::Vector3d mav;
geometry_msgs::Point mav_pose;

void getMAV(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    mav_pose.x = pose->pose.position.x;
    mav_pose.y = pose->pose.position.y;
    mav_pose.z = pose->pose.position.z;
}

// R: [x-axis 180]
// T: [x, y, z] = [] (measured)
void frameTransfer(const geometry_msgs::Point::ConstPtr& point) {
	Eigen::Matrix3d R1, R2;
        Eigen::Vector3d T(-0.04, 0.1, -0.1);
        float theta_x = -180*M_PI/180;
        float theta_z = 90*M_PI/180;
        R1 << 1,             0,            0,
              0,  cos(theta_x), sin(theta_x),
              0, -sin(theta_x), cos(theta_x);
        R2 << cos(theta_z), sin(theta_z), 0,
             -sin(theta_z), cos(theta_z), 0,
                         0,            0, 1;

	pos << point -> x, point -> y, point -> z;
    mav << mav_pose.x, mav_pose.y, mav_pose.z;
        trans_pos = R1*pos+T+mav;

	position.x = trans_pos.x();
	position.y = trans_pos.y();
	position.z = trans_pos.z();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "camera_extrinsics");
	ros::NodeHandle nh;
	ros::Subscriber sub_cam2disk_pos;
    ros::Subscriber sub_mav1_pos;
	ros::Publisher pub_world2disk_pos;
	
	sub_cam2disk_pos = nh.subscribe("camera/disk_position", 30, frameTransfer);
    sub_mav1_pos = nh.subscribe("/vrpn_client_node/MAV1/pose", 30, getMAV);
	ros::Rate r(30);

	while(ros::ok())
	{
		pub_world2disk_pos = nh.advertise<geometry_msgs::Point>("world/disk_position", 100);
		pub_world2disk_pos.publish(position);

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
