/*
 * Paths.hpp
 *
 *  Created on: May 24, 2018
 *      Author: yusaku
 */

#pragma once

#include <ros/ros.h>
//#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

class PathsCR
{
private:
	geometry_msgs::Pose _cr_sz;
	geometry_msgs::Pose _cr_pp1;
	geometry_msgs::Pose _cr_pp2;
	geometry_msgs::Pose _cr_dp1;
	geometry_msgs::Pose _cr_dp2;

	nav_msgs::Path _cr_path_sz_to_pp1;
	nav_msgs::Path _cr_path_pp1_to_dp1;
	nav_msgs::Path _cr_path_dp1_to_pp2;
	nav_msgs::Path _cr_path_pp2_to_dp2;

	void applyCubicMotion(nav_msgs::Path &path);
	void applyCubicRotation(nav_msgs::Path &path);

	static const PathsCR * const instance;

public:
	PathsCR(void);

	static inline const PathsCR * const GetInstance(void)
	{
		return instance;
	}

	inline geometry_msgs::Pose get_cr_sz(void) const
	{
		return this->_cr_sz;
	}
	inline geometry_msgs::Pose get_cr_pp1(void) const
	{
		return this->_cr_pp1;
	}
	inline geometry_msgs::Pose get_cr_pp2(void) const
	{
		return this->_cr_pp2;
	}
	inline geometry_msgs::Pose get_cr_dp1(void) const
	{
		return this->_cr_dp1;
	}
	inline geometry_msgs::Pose get_cr_dp2(void) const
	{
		return this->_cr_dp2;
	}

	inline nav_msgs::Path get_cr_path_sz_to_pp1(void) const
	{
		return this->_cr_path_sz_to_pp1;
	}
	inline nav_msgs::Path get_cr_path_pp1_to_dp1(void) const
	{
		return this->_cr_path_pp1_to_dp1;
	}
	inline nav_msgs::Path get_cr_path_dp1_to_pp2(void) const
	{
		return this->_cr_path_dp1_to_pp2;
	}
	inline nav_msgs::Path get_cr_path_pp2_to_dp2(void) const
	{
		return this->_cr_path_pp2_to_dp2;
	}
};

const PathsCR * const PathsCR::instance = new PathsCR();

PathsCR::PathsCR(void)
{
	// it's supposed to be like this by design:
	static const geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0.0);
	geometry_msgs::PoseStamped poseStamped;
	poseStamped.header.frame_id = "map";

	/*
	 * Field Coordinates for Carrying Robot (CR)
	 */

	this->_cr_sz.position.x = 0.553;
	this->_cr_sz.position.y = 9.547;
	this->_cr_sz.position.z = 0.000;
	this->_cr_sz.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_pp1.position.x =  0.480;
	this->_cr_pp1.position.y = 11.900;
	this->_cr_pp1.position.z =  0.000;
	this->_cr_pp1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_pp2.position.x = this->_cr_pp1.position.x + 0.150;
	this->_cr_pp2.position.y = this->_cr_pp1.position.y;
	this->_cr_pp2.position.z =  0.000;
	this->_cr_pp2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_dp1.position.x = 1.750;
	this->_cr_dp1.position.y = 3.500;
	this->_cr_dp1.position.z = 0.000;
	this->_cr_dp1.orientation = orientation;

	this->_cr_dp2.position.x = 1.500;
	this->_cr_dp2.position.y = 1.400;
	this->_cr_dp2.position.z = 0.000;
	this->_cr_dp2.orientation = orientation;

	/**************************************
	 * path 0
	 * for SZ -> PP1
	 **************************************/
	this->_cr_path_sz_to_pp1.header.frame_id = "map";
	this->_cr_path_sz_to_pp1.poses.clear();

	poseStamped.pose = this->_cr_sz;
	this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.483;
	poseStamped.pose.position.y = 10.333;
	this->_cr_pp1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.467;
	poseStamped.pose.position.y = 11.167;
	this->_cr_pp1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	poseStamped.pose = this->_cr_pp1;
	this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	//poseStamped.pose = this->_cr_dp1;
	//this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	/**************************************
	 * waypoint 1
	 * for LZ (PP1) -> DP1
	 **************************************/
	this->_cr_path_pp1_to_dp1.header.frame_id = "map";
	this->_cr_path_pp1_to_dp1.poses.clear();

	poseStamped.pose = this->_cr_pp1;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  this->_cr_pp1.position.x;
	poseStamped.pose.position.y = 11.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 88.8 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.625;
	poseStamped.pose.position.y = 10.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 81.6 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.800;
	poseStamped.pose.position.y =  9.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 66.3 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.000;
	poseStamped.pose.position.y =  8.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 45.0 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.200;
	poseStamped.pose.position.y =  7.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 23.7 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.350;
	poseStamped.pose.position.y =  6.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 8.43 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.450;
	poseStamped.pose.position.y =  5.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 1.23 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x = 1.625;
	poseStamped.pose.position.y = 4.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.0 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose = this->_cr_dp1;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	/**************************************
	 * path 2
	 * for DP1 -> PP2
	 **************************************/
	this->_cr_path_dp1_to_pp2.header.frame_id = "map";
	this->_cr_path_dp1_to_pp2.poses.clear();

	//poseStamped.pose = this->_cr_dp1;
	//this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

//	poseStamped.pose.position.x = 1.250;
	poseStamped.pose.position.x = 0.750;
	poseStamped.pose.position.y = 4.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.0 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.000;
	poseStamped.pose.position.y =  5.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 1.23 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.900;
	poseStamped.pose.position.y =  6.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 8.43 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.800;
	poseStamped.pose.position.y =  7.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 23.7 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.750;
	poseStamped.pose.position.y =  8.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 45.0 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.675;
	poseStamped.pose.position.y =  9.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 66.3 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.625;
	poseStamped.pose.position.y = 10.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 81.6 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  this->_cr_pp2.position.x;
	poseStamped.pose.position.y = 11.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 88.8 / 180.0);
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	poseStamped.pose = this->_cr_pp2;
	this->_cr_path_dp1_to_pp2.poses.push_back(poseStamped);

	/**************************************
	 * path 3
	 * for PP2 -> DP2
	 **************************************/
	this->_cr_path_pp2_to_dp2.header.frame_id = "map";
	this->_cr_path_pp2_to_dp2.poses.clear();

	//poseStamped.pose = this->_cr_pp2;
	//this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  this->_cr_pp2.position.x;
	poseStamped.pose.position.y = 11.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 88.8 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.675;
	poseStamped.pose.position.y = 10.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 81.6 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.750;
	poseStamped.pose.position.y =  9.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 66.3 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.850;
	poseStamped.pose.position.y =  8.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 45.0 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.000;
	poseStamped.pose.position.y =  7.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 23.7 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.200;
	poseStamped.pose.position.y =  6.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 8.43 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.300;
	poseStamped.pose.position.y =  5.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 1.23 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x = 1.400;
	poseStamped.pose.position.y = 4.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.0 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x = 1.450;
	poseStamped.pose.position.y = 3.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.0 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose.position.x = 1.500;
	poseStamped.pose.position.y = 2.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.0 / 180.0);
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);

	poseStamped.pose = this->_cr_dp2;
	this->_cr_path_pp2_to_dp2.poses.push_back(poseStamped);



	this->applyCubicMotion(this->_cr_path_pp1_to_dp1);
	this->applyCubicMotion(this->_cr_path_dp1_to_pp2);
	this->applyCubicRotation(this->_cr_path_pp1_to_dp1);
	this->applyCubicRotation(this->_cr_path_dp1_to_pp2);
	this->applyCubicRotation(this->_cr_path_pp2_to_dp2);
}

void PathsCR::applyCubicMotion(nav_msgs::Path &path)
{
	int T = path.poses.size() - 1;

	double initial_y = path.poses.at(0).pose.position.y;
	double final_y   = path.poses.at(T).pose.position.y;
	double y_m = final_y - initial_y;
	double d_y = y_m / T;

	double initial_x = path.poses.at(0).pose.position.x;
	double final_x   = path.poses.at(T).pose.position.x;
	double x_m = final_x - initial_x;

	double a_on_two = -x_m / pow(y_m, 3);

	for(int i = 0; i <= T; i++)
	{
		double y = d_y * i;
		double x = initial_x - (a_on_two * ((3.0 * y_m) - (2.0 * y)) * pow(y, 2));
		y += initial_y;

		path.poses.at(i).pose.position.y = y;
		path.poses.at(i).pose.position.x = x;
	}
}

void PathsCR::applyCubicRotation(nav_msgs::Path &path)
{
	int T = path.poses.size() - 1;
	tf::Quaternion q_i(path.poses.at(0).pose.orientation.x,
			path.poses.at(0).pose.orientation.y,
			path.poses.at(0).pose.orientation.z,
			path.poses.at(0).pose.orientation.w);
	double initial_z = tf::getYaw(q_i);

	tf::Quaternion q_f(path.poses.at(T).pose.orientation.x,
			path.poses.at(T).pose.orientation.y,
			path.poses.at(T).pose.orientation.z,
			path.poses.at(T).pose.orientation.w);
	double final_z = tf::getYaw(q_f);
	double theta_m = final_z - initial_z;
	double alpha_m = 6.0 * theta_m / (T * T);

	for(int i = 0; i <= T; i++)
	{
		double theta = initial_z + ( alpha_m * ((pow(i, 2) / 2.0) - (pow(i, 3) / (3.0 * T))));
		path.poses.at(i).pose.orientation = tf::createQuaternionMsgFromYaw(theta);
	}
}




