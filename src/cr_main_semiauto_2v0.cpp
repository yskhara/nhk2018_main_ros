/*
 * tr_main_auto_2v0.cpp
 *
 *  Created on: Mar 22, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include<tf/transform_listener.h>
#include <vector>
#include <string>

#include "Coordinates.hpp"

enum class CRControllerStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,

	standby = 0x0010,				// standing by at start zone
	moving,							// moving around
	motion_cplt,					// motion completed; transitioning

	pp_pickingup,					// picking up shuttles at a PP
	delivering_right,				// delivering a shuttle
	delivering_left,				// delivering a shuttle
};

enum class CRControllerCommands : uint16_t
{
	shutdown,						// shutdown

	standby,						// stand-by at SZ

	pp_pickup,						// pick up shuttles at a PP

	deliver_right,
	deliver_left,

	sz_to_pp1,						// move from SZ  to PP1
	sz_to_pp2,						// move from SZ  to PP2
	pp1_to_dp1,						// move from PP1 to DP1
	pp2_to_dp2,						// move from PP2 to DP2
	dp1_to_pp2,						// move from DP1 to PP2

	disarm,
	delay,

	segno,
	dal_segno,
};

enum class CarrierStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,

	sensing				= 0x0020,

	//operational			= 0x0010,
	disarmed			= 0x0010,
	pickedup			= 0x0011,
	delivered_r			= 0x0012,
	delivered_l			= 0x0013,

	delivering_r		= 0x0081,
	delivering_l		= 0x0082,
};

enum class CarrierCommands : uint16_t
{
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,

	disarm_cmd			= 0x0010,
	pickup_cmd			= 0x0011,

	deliver_r_cmd		= 0x0020,
	deliver_l_cmd		= 0x0021,
};

enum class BaseStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,
	operational			= 0x0010,
};

enum class BaseCommands : uint16_t
{
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,
	operational_cmd		= 0x0010,
};

class CrMain
{
public:
	CrMain(void);

private:
	void baseStatusCallback(const std_msgs::UInt16::ConstPtr& msg);
	void handStatusCallback(const std_msgs::UInt16::ConstPtr& msg);
	//void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg);

	void control_timer_callback(const ros::TimerEvent& event);

	ros::NodeHandle nh_;

	ros::Subscriber hand_status_sub;
	ros::Publisher hand_cmd_pub;
	std_msgs::UInt16 hand_cmd_msg;

	ros::Subscriber base_status_sub;
	ros::Publisher base_cmd_pub;
	std_msgs::UInt16 base_cmd_msg;

	ros::Subscriber goal_reached_sub;

	ros::Subscriber joy_sub;

	ros::Publisher target_pub;
	ros::Publisher abort_pub;

	//nav_msgs::Path target_msg;
	std_msgs::Bool abort_msg;
	tf::TransformListener _tflistener;

	ros::Publisher initialpose_pub;
	//geometry_msgs::PoseWithCovarianceStamped initialpose_msg;

	int currentCommandIndex = -1;
	std::vector<CRControllerCommands> command_list;
	CRControllerStatus _status;

	double _amt_coeff;
	double _target_x = 0.0;
	double _target_y = 0.0;
	double _target_yaw = M_PI / 2.0;
	bool _rush = false;

	//double _receive_delay_s;

	double _delay_s = 0.0;

	ros::Timer control_timer;

	CarrierStatus hand_last_status = CarrierStatus::shutdown;
	ros::Time hand_last_status_time;

	BaseStatus base_last_status = BaseStatus::shutdown;
	ros::Time base_last_status_time;

	void shutdown(void);
	void restart(void);

	void amt(void);
	void disarm(void);
	void pickup(void);
	void deliver_r(void);
	void deliver_l(void);

	void set_pose(geometry_msgs::Pose pose);
	void publish_path(nav_msgs::Path path);
	void publish_path(geometry_msgs::Pose to);
	void publish_path(geometry_msgs::Pose from, geometry_msgs::Pose to);
	void publish_path(geometry_msgs::Pose from, geometry_msgs::Pose via, geometry_msgs::Pose to);

	void set_delay(double delay_s);

	// flags
	//bool _start_pressed = false;
	bool _next_pressed = false;
	bool _abort_pressed = false;
	//bool _is_moving = false;
	//bool _is_throwing = false;
	bool _delivery_cplt_right = false;
	bool _delivery_cplt_left = false;
	bool _is_manual_enabled = false;
	bool _goal_reached = false;
	bool _has_base_restarted = false;

	inline void clear_flags(void)
	{
		//_start_pressed = false;
		_next_pressed = false;
		_abort_pressed = false;
		//_is_moving = false;
		//_is_throwing = false;
		_delivery_cplt_right = false;
		_delivery_cplt_left = false;
		_is_manual_enabled = false;
		_goal_reached = false;
		_has_base_restarted = false;
	}

	static int ButtonA;
	static int ButtonB;
	static int ButtonX;
	static int ButtonY;
	static int ButtonLB;
	static int ButtonRB;
	static int ButtonSelect;
	static int ButtonStart;
	static int ButtonLeftThumb;
	static int ButtonRightThumb;

	static int AxisDPadX;
	static int AxisDPadY;
	static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
};

int CrMain::ButtonA = 0;
int CrMain::ButtonB = 1;
int CrMain::ButtonX = 2;
int CrMain::ButtonY = 3;
int CrMain::ButtonLB = 4;
int CrMain::ButtonRB = 5;
int CrMain::ButtonSelect = 6;
int CrMain::ButtonStart = 7;
int CrMain::ButtonLeftThumb = 8;
int CrMain::ButtonRightThumb = 9;

int CrMain::AxisDPadX = 6;
int CrMain::AxisDPadY = 7;
int CrMain::AxisLeftThumbX = 0;
int CrMain::AxisLeftThumbY = 1;
int CrMain::AxisRightThumbX = 2;

CrMain::CrMain(void)
{
	this->hand_status_sub = nh_.subscribe<std_msgs::UInt16>("hand/status", 10, &CrMain::handStatusCallback, this);
	this->base_status_sub = nh_.subscribe<std_msgs::UInt16>("base/status", 10, &CrMain::baseStatusCallback, this);
	joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CrMain::joyCallback, this);
	//this->shutdown_sub = nh_.subscribe<std_msgs::Bool>("shutdown", 10, &TrMain::shutdownCallback, this);

	this->hand_cmd_pub = nh_.advertise<std_msgs::UInt16>("hand/cmd", 10);
	this->base_cmd_pub = nh_.advertise<std_msgs::UInt16>("base/cmd", 10);

	this->goal_reached_sub = nh_.subscribe<std_msgs::Bool>("goal_reached", 10, &CrMain::goalReachedCallback, this);
	this->target_pub = nh_.advertise<nav_msgs::Path>("target_path", 10);
	this->abort_pub = nh_.advertise<std_msgs::Bool>("abort", 10);

	this->initialpose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

	auto private_nh = ros::NodeHandle("~");
	private_nh.param("amt_coeff", this->_amt_coeff, 0.25);

	nh_.getParam("ButtonA", ButtonA);
	nh_.getParam("ButtonB", ButtonB);
	nh_.getParam("ButtonX", ButtonX);
	nh_.getParam("ButtonY", ButtonY);
	nh_.getParam("ButtonLB", ButtonLB);
	nh_.getParam("ButtonRB", ButtonRB);
	nh_.getParam("ButtonSelect", ButtonSelect);
	nh_.getParam("ButtonStart", ButtonStart);
	nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);
	nh_.getParam("ButtonRightThumb", ButtonRightThumb);

	nh_.getParam("AxisDPadX", AxisDPadX);
	nh_.getParam("AxisDPadY", AxisDPadY);
	nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
	nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
	nh_.getParam("AxisRightThumbX", AxisRightThumbX);



	/*
	 * Initialize Control Sequence
	 */

	this->command_list.clear();
	this->command_list.push_back(CRControllerCommands::standby);

//#define UNIT_TEST
//#define TZ1_TEST
#define FULL_OP

#ifdef UNIT_TEST
	// repeat from here
	this->command_list.push_back(CRControllerCommands::segno);

	// pick up test
	this->command_list.push_back(CRControllerCommands::pp_pickup);

	// delivery test
	this->command_list.push_back(CRControllerCommands::dp1_deliver_tz1);
	this->command_list.push_back(CRControllerCommands::dp1_deliver_tz2);

	// repeat from here
	this->command_list.push_back(CRControllerCommands::dal_segno);
#endif

#ifdef TZ1_TEST
	// receive at dp1
	this->command_list.push_back(CRControllerCommands::sz_to_dp1);

	// repeat from here
	this->command_list.push_back(CRControllerCommands::segno);

	this->command_list.push_back(CRControllerCommands::dp_receive);
	this->command_list.push_back(CRControllerCommands::delay);

	// throw at tz1
	this->command_list.push_back(CRControllerCommands::dp1_to_tz1);
	this->command_list.push_back(CRControllerCommands::set_tz1);
	this->command_list.push_back(CRControllerCommands::tz_throw);
	this->command_list.push_back(CRControllerCommands::disarm);

	// receive at dp1
	this->command_list.push_back(CRControllerCommands::tz1_to_dp1);

	// repeat forever
	this->command_list.push_back(CRControllerCommands::dal_segno);
#endif

#ifdef FULL_OP
	// pickup at pp1
	this->command_list.push_back(CRControllerCommands::disarm);
	this->command_list.push_back(CRControllerCommands::sz_to_pp1);
	this->command_list.push_back(CRControllerCommands::pp_pickup);

	// deliver at dp1 for tz1
	this->command_list.push_back(CRControllerCommands::pp1_to_dp1);
	this->command_list.push_back(CRControllerCommands::deliver_right);

	this->command_list.push_back(CRControllerCommands::deliver_left);

	// pickup at pp2
	this->command_list.push_back(CRControllerCommands::disarm);
	this->command_list.push_back(CRControllerCommands::dp1_to_pp2);
	this->command_list.push_back(CRControllerCommands::pp_pickup);

	// deliver at dp2 for tz3
	this->command_list.push_back(CRControllerCommands::pp2_to_dp2);
	this->command_list.push_back(CRControllerCommands::deliver_right);
	this->command_list.push_back(CRControllerCommands::deliver_left);
	this->command_list.push_back(CRControllerCommands::pp_pickup);
	this->command_list.push_back(CRControllerCommands::shutdown);
#endif

	this->_status = CRControllerStatus::shutdown;

	// timer starts immediately
	control_timer = nh_.createTimer(ros::Duration(0.1), &CrMain::control_timer_callback, this);
}

void CrMain::baseStatusCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	BaseStatus status = (BaseStatus)msg->data;

	switch(status)
	{
	case BaseStatus::shutdown:
		if(this->base_last_status != BaseStatus::shutdown)
		{
			this->shutdown();
		}
		break;

	case BaseStatus::reset:
		if(this->base_last_status == BaseStatus::shutdown)
		{
			//this->restart();
			this->_status = CRControllerStatus::reset;
			this->currentCommandIndex = 0;
			ROS_INFO("restarting.");

			this->_has_base_restarted = true;
		}
		break;

	default:
		break;
	}

	base_last_status = status;
	base_last_status_time = ros::Time::now();
}

void CrMain::handStatusCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	CarrierStatus status = (CarrierStatus)msg->data;

	switch(status)
	{
	case CarrierStatus::shutdown:
		break;

	case CarrierStatus::reset:
		break;

	case CarrierStatus::delivering_r:
		this->_delivery_cplt_right = false;
		break;

	case CarrierStatus::delivered_r:
		this->_delivery_cplt_right = true;
		break;

	case CarrierStatus::delivering_l:
		this->_delivery_cplt_left = false;
		break;

	case CarrierStatus::delivered_l:
		this->_delivery_cplt_left = true;
		break;


	default:
		break;
	}

	hand_last_status = status;
	hand_last_status_time = ros::Time::now();
}

void CrMain::goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(!msg->data)
	{
		// not reached yet
		return;
	}

	//ROS_INFO("goal reached.");

	abort_msg.data = true;
	this->abort_pub.publish(abort_msg);

	this->_goal_reached = true;
}

void CrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	static bool last_a = false;
	static bool last_b = false;
	static bool last_x = false;
	static bool last_y = false;
	static int last_dpadX = 0;

	bool _a = joy->buttons[ButtonA];
	bool _b = joy->buttons[ButtonB];
	bool _x = joy->buttons[ButtonX];
	bool _y = joy->buttons[ButtonY];

	bool _start = joy->buttons[ButtonStart];

	if(_start)
	{
		this->shutdown();
	}

	if(_a && !last_a)
	{
		this->_next_pressed = true;
	}
	else if(_b && !last_b)
	{
		this->_abort_pressed = true;
	}
	else if(_x && !last_x)
	{

	}
	else if(_y && !last_y)
	{

	}

	int dpadX = -joy->axes[AxisDPadX];
	if(dpadX != last_dpadX)
	{
		if(0 < dpadX)
		{
			if(this->_is_manual_enabled)
			{
				this->_target_yaw = 0.0;
			}
		}
		else if(dpadX < 0)
		{
			if(this->_is_manual_enabled)
			{
				this->_target_yaw = M_PI / 2.0;
			}
		}
	}

	last_a = _a;
	last_b = _b;
	last_x = _x;
	last_y = _y;

	if(this->_is_manual_enabled)
	{
		//swap x and y
		this->_target_x = joy->axes[AxisLeftThumbY];
		this->_target_y = joy->axes[AxisLeftThumbX];
		this->_rush = (joy->buttons[ButtonRB] != 0);
	}
	else
	{
		this->_target_x = 0.0;
		this->_target_y = 0.0;
		//this->_target_yaw = 0.0;
		this->_rush = false;
	}
}

void CrMain::shutdown(void)
{
	if(this->currentCommandIndex != -1)
	{
		ROS_INFO("aborting.");

		this->currentCommandIndex = -1;

		//this->unchuck_all();
	}

	abort_msg.data = true;
	this->abort_pub.publish(abort_msg);

	hand_cmd_msg.data = (uint16_t)CarrierCommands::shutdown_cmd;
	hand_cmd_pub.publish(hand_cmd_msg);

	base_cmd_msg.data = (uint16_t)BaseCommands::shutdown_cmd;
	base_cmd_pub.publish(base_cmd_msg);

	clear_flags();

	this->_status = CRControllerStatus::shutdown;
}

void CrMain::restart(void)
{

	hand_cmd_msg.data = (uint16_t)CarrierCommands::reset_cmd;
	hand_cmd_pub.publish(hand_cmd_msg);

	base_cmd_msg.data = (uint16_t)BaseCommands::reset_cmd;
	base_cmd_pub.publish(base_cmd_msg);

	base_cmd_msg.data = (uint16_t)BaseCommands::operational_cmd;
	base_cmd_pub.publish(base_cmd_msg);

	//this->unchuck_all();

	//this->clear_flags();
}

void CrMain::set_pose(geometry_msgs::Pose pose)
{
	geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
	// assuming that the robot is at start position
	initialpose_msg.header.frame_id = "map";
	initialpose_msg.header.stamp = ros::Time::now();
	initialpose_msg.pose.pose = pose;
	initialpose_msg.pose.covariance =
	{
		0.025, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.025, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.006853891945200942
	};

	this->initialpose_pub.publish(initialpose_msg);
}

void CrMain::publish_path(nav_msgs::Path path)
{
	//this->set_pose(path.poses.at(0).pose);

	this->target_pub.publish(path);

	this->_target_yaw = tf::getYaw(path.poses.back().pose.orientation);
}

void CrMain::publish_path(geometry_msgs::Pose to)
{
	nav_msgs::Path path_msg;
	path_msg.poses.clear();

	geometry_msgs::PoseStamped _pose;

	path_msg.header.frame_id = "map";
	path_msg.header.stamp = ros::Time::now();

	_pose.header.frame_id = "map";
	_pose.header.stamp = ros::Time::now();
	_pose.pose = to;
	path_msg.poses.push_back(_pose);

	this->publish_path(path_msg);
}

void CrMain::publish_path(geometry_msgs::Pose from, geometry_msgs::Pose to)
{
	nav_msgs::Path path_msg;
	path_msg.poses.clear();

	geometry_msgs::PoseStamped _pose;

	path_msg.header.frame_id = "map";
	path_msg.header.stamp = ros::Time::now();

	_pose.header.frame_id = "map";
	_pose.header.stamp = ros::Time::now();
	_pose.pose = from;
	path_msg.poses.push_back(_pose);

	_pose.pose = to;
	path_msg.poses.push_back(_pose);

	this->publish_path(path_msg);
}

void CrMain::publish_path(geometry_msgs::Pose from, geometry_msgs::Pose via, geometry_msgs::Pose to)
{
	nav_msgs::Path path_msg;
	path_msg.poses.clear();

	geometry_msgs::PoseStamped _pose;

	path_msg.header.frame_id = "map";
	path_msg.header.stamp = ros::Time::now();

	_pose.header.frame_id = "map";
	_pose.header.stamp = ros::Time::now();
	_pose.pose = from;
	path_msg.poses.push_back(_pose);

	_pose.pose = via;
	path_msg.poses.push_back(_pose);

	_pose.pose = to;
	path_msg.poses.push_back(_pose);

	this->publish_path(path_msg);
}

void CrMain::control_timer_callback(const ros::TimerEvent& event)
{
	this->publish_path(Coordinates::GetInstance()->get_cr_path_pp1_to_dp1());

	if(this->command_list.size() <= this->currentCommandIndex)
	{
		this->shutdown();

		return;
	}

	if(this->_status == CRControllerStatus::shutdown)
	{
		return;
	}

	if(this->currentCommandIndex == -1)
	{
		this->currentCommandIndex = 0;
	}

	CRControllerCommands currentCommand = this->command_list.at(this->currentCommandIndex);

	if(currentCommand == CRControllerCommands::shutdown)
	{
		this->shutdown();
	}
	else if(currentCommand == CRControllerCommands::standby)
	{
		if(!this->_has_base_restarted)
		{
			return;
		}

		if(this->_status == CRControllerStatus::standby)
		{
			if(this->_next_pressed)
			{
				this->restart();

				//this->unchuck_all();

				clear_flags();
				this->_status = CRControllerStatus::motion_cplt;

				this->currentCommandIndex++;
				ROS_INFO("starting.");
			}
		}
		else
		{
			//this->unchuck_all();

			set_pose(Coordinates::GetInstance()->get_cr_sz());

			this->_next_pressed = false;
			//clear_flags();
			this->_status = CRControllerStatus::standby;

			//this->sense();
			ROS_INFO("standing by.");
		}
	}
	else if(currentCommand == CRControllerCommands::sz_to_pp1)
	{
		if(this->_status == CRControllerStatus::moving)
		{
			if(this->_next_pressed)
			{
				clear_flags();
				this->_next_pressed = true;
				this->_status = CRControllerStatus::pp_pickingup;

				this->currentCommandIndex++;
				ROS_INFO("goal reached : pp1 (operator input)");
				//ROS_INFO("picked up shuttles.");
			}
			else if(this->_abort_pressed || this->_goal_reached)
			{
				//chuck_all();

				clear_flags();
				this->_status = CRControllerStatus::motion_cplt;
				//this->_status = CRControllerStatus::pp_pickingup;

				this->currentCommandIndex++;
				ROS_INFO("goal reached : pp1");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_cr_sz(), Coordinates::GetInstance()->get_cr_wp0(), Coordinates::GetInstance()->get_cr_pp1());

			clear_flags();
			this->_status = CRControllerStatus::moving;

			this->disarm();

			ROS_INFO("moving : sz -> pp1");
		}
	}
	else if(currentCommand == CRControllerCommands::pp1_to_dp1)
	{
		if(this->_status == CRControllerStatus::moving)
		{
			if(this->_abort_pressed || this->_goal_reached)
			{
				clear_flags();
				this->_status = CRControllerStatus::motion_cplt;

				this->currentCommandIndex++;
				ROS_INFO("goal reached : dp1");
			}
		}
		else
		{
			this->publish_path(Coordinates::GetInstance()->get_cr_path_pp1_to_dp1());

			clear_flags();
			this->_status = CRControllerStatus::moving;

			ROS_INFO("moving : pp1 -> dp1");
		}
	}
	else if(currentCommand == CRControllerCommands::pp2_to_dp2)
	{
		if(this->_status == CRControllerStatus::moving)
		{
			if(this->_abort_pressed || this->_goal_reached)
			{
				clear_flags();
				this->_status = CRControllerStatus::motion_cplt;

				this->currentCommandIndex++;
				ROS_INFO("goal reached : dp2");
			}
		}
		else
		{
			nav_msgs::Path path_msg;
			path_msg.poses.clear();

			geometry_msgs::PoseStamped _pose;

			path_msg.header.frame_id = "map";
			path_msg.header.stamp = ros::Time::now();

			_pose.header.frame_id = "map";
			_pose.header.stamp = ros::Time::now();
			_pose.pose = Coordinates::GetInstance()->get_cr_pp2();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_wp3_1();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_wp3_2();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_wp3_3();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_dp2();
			path_msg.poses.push_back(_pose);

			this->publish_path(path_msg);

			clear_flags();
			this->_status = CRControllerStatus::moving;

			ROS_INFO("moving : pp2 -> dp2");
		}
	}
	else if(currentCommand == CRControllerCommands::dp1_to_pp2)
	{
		if(this->_status == CRControllerStatus::moving)
		{
			if(this->_next_pressed)
			{
				//chuck_all();

				// maybe it's a bad idea...
				//clear_flags();
				this->_status = CRControllerStatus::pp_pickingup;

				this->currentCommandIndex++;
				ROS_INFO("goal reached : pp2 (operator input)");
				//ROS_INFO("picked up shuttles.");
			}
			else if(this->_abort_pressed || this->_goal_reached)
			{
				clear_flags();
				this->_status = CRControllerStatus::motion_cplt;

				this->currentCommandIndex++;
				ROS_INFO("goal reached : pp2");
			}
		}
		else
		{
			nav_msgs::Path path_msg;
			path_msg.poses.clear();

			geometry_msgs::PoseStamped _pose;

			path_msg.header.frame_id = "map";
			path_msg.header.stamp = ros::Time::now();

			_pose.header.frame_id = "map";
			_pose.header.stamp = ros::Time::now();
			_pose.pose = Coordinates::GetInstance()->get_cr_dp1();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_wp2_0();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_wp2_1();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_wp2_2();
			path_msg.poses.push_back(_pose);

			_pose.pose = Coordinates::GetInstance()->get_cr_pp2();
			path_msg.poses.push_back(_pose);

			this->publish_path(path_msg);

			clear_flags();
			this->_status = CRControllerStatus::moving;

			ROS_INFO("moving : dp1 -> pp2");
		}
	}
	else if(currentCommand == CRControllerCommands::pp_pickup)
	{
		if(this->_status == CRControllerStatus::pp_pickingup)
		{
			if(this->_next_pressed)
			{
				this->pickup();

				clear_flags();
				this->_status = CRControllerStatus::motion_cplt;

				this->currentCommandIndex++;
				ROS_INFO("picked up shuttles.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_status = CRControllerStatus::pp_pickingup;
			ROS_INFO("picking up shuttles.");
		}

		this->amt();
	}
	else if(currentCommand == CRControllerCommands::deliver_right)
	{
		if(this->_status == CRControllerStatus::delivering_right)
		{
			if(this->_delivery_cplt_right)
			{
				// delivery completed

				clear_flags();

				this->currentCommandIndex++;
				ROS_INFO("shuttle delivered: right");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_status = CRControllerStatus::delivering_right;

			this->deliver_r();
			this->_delivery_cplt_right = false;

			ROS_INFO("shuttle delivering: right");
		}

		this->amt();
	}
	else if(currentCommand == CRControllerCommands::deliver_left)
	{
		if(this->_status == CRControllerStatus::delivering_left)
		{
			if(this->_delivery_cplt_left)
			{
				// delivery completed

				clear_flags();

				this->currentCommandIndex++;
				ROS_INFO("shuttle delivered: left");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_status = CRControllerStatus::delivering_left;

			this->deliver_l();
			this->_delivery_cplt_left = false;

			ROS_INFO("shuttle delivering: left");
		}

		this->amt();
	}
	else if(currentCommand == CRControllerCommands::delay)
	{
		if(this->_delay_s == 0)
		{
			return;
		}

		if(this->_delay_s < ros::Time::now().toSec())
		{
			this->_delay_s = 0;
			this->currentCommandIndex++;
		}
	}
	else if(currentCommand == CRControllerCommands::disarm)
	{
		this->disarm();
		this->currentCommandIndex++;
	}
	else if(currentCommand == CRControllerCommands::segno)
	{
		this->currentCommandIndex++;
	}
	else if(currentCommand == CRControllerCommands::dal_segno)
	{
		auto segno_iter = std::find(this->command_list.begin(), this->command_list.end(), CRControllerCommands::segno);
		if(segno_iter == this->command_list.end())
		{
			// abort on error
			this->shutdown();
		}
		auto segno_index = std::distance(this->command_list.begin(), segno_iter);
		this->currentCommandIndex = segno_index;
	}
}

void CrMain::amt(void)
{
	if(!this->_is_manual_enabled)
	{
		return;
	}

	geometry_msgs::Pose moving_target;
	tf::StampedTransform base_link;

	try
	{
		this->_tflistener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
		this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
	}
	catch(...)
	{
		return;
	}

	double coeff = 0.0;
	if(this->_rush)
	{
		// rushing mode
		coeff = this->_amt_coeff * 10;
	}
	else
	{
		// precision mode
		coeff = this->_amt_coeff;
	}
	moving_target.position.x = base_link.getOrigin().x() + (this->_target_x * coeff);
	moving_target.position.y = base_link.getOrigin().y() + (this->_target_y * coeff);
	moving_target.orientation = tf::createQuaternionMsgFromYaw(this->_target_yaw);

	this->publish_path(moving_target);
}

void CrMain::disarm(void)
{
	// unchuck all
	hand_cmd_msg.data = (uint16_t)CarrierCommands::disarm_cmd;
	hand_cmd_pub.publish(hand_cmd_msg);
}

void CrMain::pickup(void)
{
	// chuck all
	hand_cmd_msg.data = (uint16_t)CarrierCommands::pickup_cmd;
	hand_cmd_pub.publish(hand_cmd_msg);
}

void CrMain::deliver_r(void)
{
	// deliver 1
	hand_cmd_msg.data = (uint16_t)CarrierCommands::deliver_r_cmd;
	hand_cmd_pub.publish(hand_cmd_msg);
}

void CrMain::deliver_l(void)
{
	// deliver 2
	hand_cmd_msg.data = (uint16_t)CarrierCommands::deliver_l_cmd;
	hand_cmd_pub.publish(hand_cmd_msg);
}

void CrMain::set_delay(double delay_s)
{
	//if(this->_delay_s != 0)
	//{
	//	return;
	//}

	this->_delay_s = ros::Time::now().toSec() + delay_s;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tr_main");

	CrMain *instance = new CrMain();
	ROS_INFO("CR main node has started.");

	ros::spin();
	ROS_INFO("CR main node has been terminated.");
}



