/*
 * Coordinates.hpp
 *
 *  Created on: Mar 21, 2018
 *      Author: yusaku
 */

#pragma once


#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

class Coordinates
{
private:
	geometry_msgs::Pose _tr_sz;
	geometry_msgs::Pose _tr_wp1;
	geometry_msgs::Pose _tr_wp2_1;
	geometry_msgs::Pose _tr_wp2_2;
	geometry_msgs::Pose _tr_wp3_1;
	geometry_msgs::Pose _tr_wp3_2;
	geometry_msgs::Pose _tr_dp1;
	geometry_msgs::Pose _tr_dp2;
	geometry_msgs::Pose _tr_tz1;
	geometry_msgs::Pose _tr_tz2;
	geometry_msgs::Pose _tr_tz3;

	geometry_msgs::Pose _cr_sz;
	geometry_msgs::Pose _cr_pp1;
	geometry_msgs::Pose _cr_pp2;
	geometry_msgs::Pose _cr_dp1;
	geometry_msgs::Pose _cr_dp2;
	geometry_msgs::Pose _cr_wp0;
	nav_msgs::Path _cr_path_pp1_to_dp1;
	geometry_msgs::Pose _cr_wp1_1;
	geometry_msgs::Pose _cr_wp1_2;
	geometry_msgs::Pose _cr_wp1_3;
	geometry_msgs::Pose _cr_wp1_4;
	geometry_msgs::Pose _cr_wp1_5;
	geometry_msgs::Pose _cr_wp1_6;
	geometry_msgs::Pose _cr_wp2_0;
	geometry_msgs::Pose _cr_wp2_1;
	geometry_msgs::Pose _cr_wp2_2;
	geometry_msgs::Pose _cr_wp3_1;
	geometry_msgs::Pose _cr_wp3_2;
	geometry_msgs::Pose _cr_wp3_3;

	static const Coordinates *instance;

public:
	Coordinates(void);

	static inline const Coordinates * const GetInstance(void)
	{
		return instance;
	}

	inline geometry_msgs::Pose get_tr_sz(void) const
	{
		return this->_tr_sz;
	}
	inline geometry_msgs::Pose get_tr_wp1(void) const
	{
		return this->_tr_wp1;
	}
	inline geometry_msgs::Pose get_tr_wp2_1(void) const
	{
		return this->_tr_wp2_1;
	}
	inline geometry_msgs::Pose get_tr_wp2_2(void) const
	{
		return this->_tr_wp2_2;
	}
	inline geometry_msgs::Pose get_tr_wp3_1(void) const
	{
		return this->_tr_wp3_1;
	}
	inline geometry_msgs::Pose get_tr_wp3_2(void) const
	{
		return this->_tr_wp3_2;
	}
	/*
	inline geometry_msgs::Pose get_tr_dp1(void) const
	{
		return this->_tr_dp1;
	}
	*/
	inline geometry_msgs::Pose get_tr_dp2(void) const
	{
		return this->_tr_dp2;
	}
	inline geometry_msgs::Pose get_tr_tz1(void) const
	{
		return this->_tr_tz1;
	}
	inline geometry_msgs::Pose get_tr_tz2(void) const
	{
		return this->_tr_tz2;
	}
	inline geometry_msgs::Pose get_tr_tz3(void) const
	{
		return this->_tr_tz3;
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
	inline geometry_msgs::Pose get_cr_wp1_1(void) const
	{
		return this->_cr_wp1_1;
	}
	inline geometry_msgs::Pose get_cr_wp1_2(void) const
	{
		return this->_cr_wp1_2;
	}
	inline geometry_msgs::Pose get_cr_wp1_3(void) const
	{
		return this->_cr_wp1_3;
	}
	inline geometry_msgs::Pose get_cr_wp1_4(void) const
	{
		return this->_cr_wp1_4;
	}
	inline geometry_msgs::Pose get_cr_wp1_5(void) const
	{
		return this->_cr_wp1_5;
	}
	inline geometry_msgs::Pose get_cr_wp1_6(void) const
	{
		return this->_cr_wp1_6;
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

const Coordinates *Coordinates::instance = new Coordinates();

Coordinates::Coordinates(void)
{
	// it's supposed to be like this by design:
	static constexpr double tz1_angle = M_PI * (78) / 180;
	static constexpr double tz2_angle = M_PI * (90 - 9) / 180;

	/*
	 * Field Coordinates for Throwing Robot (TR)
	 */

	//this->_tr_sz.position.x = 0.500;
	//this->_tr_sz.position.y = 7.550;
	//this->_tr_sz.orientation = tf::createQuaternionMsgFromYaw(-M_PI/4);
	this->_tr_sz.position.x = 0.600;
	this->_tr_sz.position.y = 7.450;
	this->_tr_sz.position.z = 0.000;
	this->_tr_sz.orientation = tf::createQuaternionMsgFromYaw(M_PI/4);

	// SZ -> TZ1
	this->_tr_wp1.position.x = 1.250;
	this->_tr_wp1.position.y = 3.600;
	this->_tr_wp1.position.z = 0.000;
	this->_tr_wp1.orientation = tf::createQuaternionMsgFromYaw(tz1_angle);

	//this->_tr_dp1.position.x = 1.065;
	//this->_tr_dp1.position.y = 2.985;
	//this->_tr_dp1.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
	//this->_tr_dp1.position.x = 1.900;
	this->_tr_dp1.position.x = 2.100;
	this->_tr_dp1.position.y = 2.750;
	this->_tr_dp1.position.z = 0.000;
	this->_tr_dp1.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// DP1 -> TZ2
	this->_tr_wp2_1.position.x = 1.500;
	this->_tr_wp2_1.position.y = 2.500;
	this->_tr_wp2_1.position.z = 0.000;
	this->_tr_wp2_1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_tr_wp2_2.position.x = 1.500;
	this->_tr_wp2_2.position.y = 1.500;
	this->_tr_wp2_2.position.z = 0.000;
	this->_tr_wp2_2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_tr_wp3_1.position.x = 6.500;
	this->_tr_wp3_1.position.y = 1.000;
	this->_tr_wp3_1.position.z = 0.000;
	this->_tr_wp3_1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_tr_wp3_2.position.x = 3.750;
	this->_tr_wp3_2.position.y = 1.100;
	this->_tr_wp3_2.position.z = 0.000;
	this->_tr_wp3_2.orientation = tf::createQuaternionMsgFromYaw(tz1_angle);

	//this->_tr_dp2.position.x = 1.065;
	//this->_tr_dp2.position.y = 0.985;
	//this->_tr_dp2.orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
	this->_tr_dp2.position.x = 2.700;
	this->_tr_dp2.position.y = 1.300;
	this->_tr_dp2.position.z = 0.000;
	this->_tr_dp2.orientation = tf::createQuaternionMsgFromYaw(tz1_angle);

	//this->_tr_tz1.position.x = 3.775;
	//this->_tr_tz1.position.y = 2.985;
	//this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz1.position.x = 2.800;
	this->_tr_tz1.position.y = 3.250;
	this->_tr_tz1.position.z = 0.000;
	//this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(M_PI * 105 / 256);
	this->_tr_tz1.orientation = tf::createQuaternionMsgFromYaw(tz1_angle);

	//this->_tr_tz2.position.x = 3.775;
	//this->_tr_tz2.position.y = 0.985;
	//this->_tr_tz2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz2.position.x = 2.800;
	this->_tr_tz2.position.y = 1.300;
	this->_tr_tz2.position.z = 0.000;
	this->_tr_tz2.orientation = tf::createQuaternionMsgFromYaw(tz2_angle);

	//this->_tr_tz3.position.x = 7.035;
	//this->_tr_tz3.position.y = 0.985;
	//this->_tr_tz3.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_tr_tz3.position.x = 7.100;
	this->_tr_tz3.position.y = 1.300;
	this->_tr_tz3.position.z = 0.000;
	this->_tr_tz3.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);


	/*
	 * Field Coordinates for Carrying Robot (CR)
	 */

	//this->_cr_sz.position.x = 0.500;
	//this->_cr_sz.position.y = 9.350;
	//this->_cr_sz.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_cr_sz.position.x = 0.600;
	this->_cr_sz.position.y = 9.350;
	this->_cr_sz.position.z = 0.000;
	this->_cr_sz.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_pp1.position.x =  0.520;
	this->_cr_pp1.position.y = 11.900;
	this->_cr_pp1.position.z =  0.000;
	this->_cr_pp1.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_pp2.position.x =  this->_cr_pp1.position.x + 0.150;
	this->_cr_pp2.position.y = 11.900;
	this->_cr_pp2.position.z =  0.000;
	this->_cr_pp2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	this->_cr_dp1.position.x = 1.500;
	this->_cr_dp1.position.y = 3.100;
	this->_cr_dp1.position.z = 0.000;
	this->_cr_dp1.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_cr_dp2.position.x = 1.500;
	this->_cr_dp2.position.y = 1.100;
	this->_cr_dp2.position.z = 0.000;
	this->_cr_dp2.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// waypoint 0
	// for SZ -> PP1
	this->_cr_wp0.position.x =  this->_cr_pp1.position.x;
	this->_cr_wp0.position.y = 11.400;
	this->_cr_wp0.position.z =  0.000;
	this->_cr_wp0.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

	/**************************************
	 * waypoint 1
	 * for LZ (PP1) -> DP1
	 **************************************/
	geometry_msgs::PoseStamped poseStamped;
	poseStamped.header.frame_id = "map";
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

	poseStamped.pose.position.x = 1.500;
	poseStamped.pose.position.y = 4.000;
	poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 0.0 / 180.0);
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	poseStamped.pose = this->_cr_dp1;
	this->_cr_path_pp1_to_dp1.poses.push_back(poseStamped);

	////////////////////////////
	// waypoint 2
	// for DP1 -> LZ (PP2)
	////////////////////////////
	this->_cr_wp2_0.position.x =  1.500;
	this->_cr_wp2_0.position.y =  5.000;
	this->_cr_wp2_0.position.z =  0.000;
	this->_cr_wp2_0.orientation = tf::createQuaternionMsgFromYaw(M_PI * 15.0 / 180.0);
	this->_cr_wp2_0.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_cr_wp2_1.position.x =  1.000;
	this->_cr_wp2_1.position.y =  8.000;
	this->_cr_wp2_1.position.z =  0.000;
	this->_cr_wp2_1.orientation = tf::createQuaternionMsgFromYaw(M_PI * 60.0 / 180.0);

	this->_cr_wp2_1.position.x =  1.000;
	this->_cr_wp2_1.position.y =  11.500;
	this->_cr_wp2_1.position.z =  0.000;
	this->_cr_wp2_1.orientation = tf::createQuaternionMsgFromYaw(0.0);

	//this->_cr_wp2_2.position.x =  0.480;
	//this->_cr_wp2_2.position.y = 12.500;
	this->_cr_wp2_2.position.x =  this->_cr_pp2.position.x;
	this->_cr_wp2_2.position.y = 10.500;
	this->_cr_wp2_2.position.z =  0.000;
	this->_cr_wp2_2.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
	this->_cr_wp2_2.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// waypoint 3
	// for LZ (PP2) -> DP2
	this->_cr_wp3_1.position.x =  this->_cr_pp2.position.x;
	this->_cr_wp3_1.position.y = 10.500;
	this->_cr_wp3_1.position.z =  0.000;
	this->_cr_wp3_1.orientation = tf::createQuaternionMsgFromYaw(M_PI * 75.0 / 180.0);
	this->_cr_wp3_1.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_cr_wp3_2.position.x =  1.250;
	this->_cr_wp3_2.position.y =  9.000;
	this->_cr_wp3_2.position.z =  0.000;
	this->_cr_wp3_2.orientation = tf::createQuaternionMsgFromYaw(M_PI * 60.0 / 180.0);
	this->_cr_wp3_2.orientation = tf::createQuaternionMsgFromYaw(0.0);

	this->_cr_wp3_3.position.x = 1.500;
	this->_cr_wp3_3.position.y = 1.500;
	this->_cr_wp3_3.position.z = 0.000;
	this->_cr_wp3_3.orientation = tf::createQuaternionMsgFromYaw(0.0);
}


