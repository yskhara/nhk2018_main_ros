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
	geometry_msgs::Pose _cr_wp0;
	nav_msgs::Path _cr_path_sz_to_pp1;
	nav_msgs::Path _cr_path_pp1_to_dp1;
	geometry_msgs::Pose _cr_wp2_0;
	geometry_msgs::Pose _cr_wp2_1;
	geometry_msgs::Pose _cr_wp2_2;
	geometry_msgs::Pose _cr_wp3_1;
	geometry_msgs::Pose _cr_wp3_2;
	geometry_msgs::Pose _cr_wp3_3;

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
	inline geometry_msgs::Pose get_cr_wp0(void) const
	{
		return this->_cr_wp0;
	}
	inline nav_msgs::Path get_cr_path_pp1_to_dp1(void) const
	{
		return this->_cr_path_pp1_to_dp1;
	}
	inline nav_msgs::Path get_cr_path_sz_to_pp1(void) const
	{
		return this->_cr_path_sz_to_pp1;
	}
	inline geometry_msgs::Pose get_cr_wp2_0(void) const
	{
		return this->_cr_wp2_0;
	}
	inline geometry_msgs::Pose get_cr_wp2_1(void) const
	{
		return this->_cr_wp2_1;
	}
	inline geometry_msgs::Pose get_cr_wp2_2(void) const
	{
		return this->_cr_wp2_2;
	}
	inline geometry_msgs::Pose get_cr_wp3_1(void) const
	{
		return this->_cr_wp3_1;
	}
	inline geometry_msgs::Pose get_cr_wp3_2(void) const
	{
		return this->_cr_wp3_2;
	}
	inline geometry_msgs::Pose get_cr_wp3_3(void) const
	{
		return this->_cr_wp3_3;
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

	this->_cr_sz.position.x = 0.600;
	this->_cr_sz.position.y = 9.350;
	this->_cr_sz.position.z = 0.000;
	this->_cr_sz.orientation = orientation;

	this->_cr_pp1.position.x =  0.520;
	this->_cr_pp1.position.y = 11.900;
	this->_cr_pp1.position.z =  0.000;
	this->_cr_pp1.orientation = orientation;

	this->_cr_pp2.position.x =  this->_cr_pp1.position.x + 0.150;
	this->_cr_pp2.position.y = 11.900;
	this->_cr_pp2.position.z =  0.000;
	this->_cr_pp2.orientation = orientation;

	this->_cr_dp1.position.x = 1.500;
	this->_cr_dp1.position.y = 3.100;
	this->_cr_dp1.position.z = 0.000;
	this->_cr_dp1.orientation = orientation;

	this->_cr_dp2.position.x = 1.500;
	this->_cr_dp2.position.y = 1.100;
	this->_cr_dp2.position.z = 0.000;
	this->_cr_dp2.orientation = orientation;

	// waypoint 0
	// for SZ -> PP1
	this->_cr_wp0.position.x =  this->_cr_pp1.position.x;
	this->_cr_wp0.position.y = 11.400;
	this->_cr_wp0.position.z =  0.000;
	this->_cr_wp0.orientation = orientation;

	/**************************************
	 * path 0
	 * for SZ -> PP1
	 **************************************/
	this->_cr_path_sz_to_pp1.header.frame_id = "map";
	this->_cr_path_sz_to_pp1.poses.clear();

	poseStamped.pose = this->_cr_sz;
	this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.600;
	poseStamped.pose.position.y = 10.350;
	poseStamped.pose.orientation = orientation;
	this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.600;
	poseStamped.pose.position.y = 11.350;
	this->_cr_path_sz_to_pp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.600;
	poseStamped.pose.position.y = 12.350;
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
	poseStamped.pose.orientation = orientation;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.625;
	poseStamped.pose.position.y = 10.000;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  0.800;
	poseStamped.pose.position.y =  9.000;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.000;
	poseStamped.pose.position.y =  8.000;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.200;
	poseStamped.pose.position.y =  7.000;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.350;
	poseStamped.pose.position.y =  6.000;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x =  1.450;
	poseStamped.pose.position.y =  5.000;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose.position.x = 1.500;
	poseStamped.pose.position.y = 4.000;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose = this->_cr_dp1;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	// waypoint 2
	// for DP1 -> LZ (PP2)
	this->_cr_wp2_0.position.x =  1.500;
	this->_cr_wp2_0.position.y =  5.000;
	this->_cr_wp2_0.orientation = orientation;

	this->_cr_wp2_1.position.x =  1.000;
	this->_cr_wp2_1.position.y =  8.000;
	this->_cr_wp2_1.orientation = orientation;

	this->_cr_wp2_1.position.x =  1.000;
	this->_cr_wp2_1.position.y =  11.500;
	this->_cr_wp2_1.orientation = orientation;

	//this->_cr_wp2_2.position.x =  0.480;
	//this->_cr_wp2_2.position.y = 12.500;
	this->_cr_wp2_2.position.x =  this->_cr_pp2.position.x;
	this->_cr_wp2_2.position.y = 10.500;
	this->_cr_wp2_2.orientation = orientation;

	// waypoint 3
	// for LZ (PP2) -> DP2
	this->_cr_wp3_1.position.x =  this->_cr_pp2.position.x;
	this->_cr_wp3_1.position.y = 10.500;
	this->_cr_wp3_1.orientation = orientation;

	this->_cr_wp3_2.position.x =  1.250;
	this->_cr_wp3_2.position.y =  9.000;
	this->_cr_wp3_2.orientation = orientation;

	this->_cr_wp3_3.position.x = 1.500;
	this->_cr_wp3_3.position.y = 1.500;
	this->_cr_wp3_3.orientation = orientation;
}





