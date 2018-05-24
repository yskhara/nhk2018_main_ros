/*
 * tr_can.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: yusaku
 */


#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>

#include <boost/array.hpp>

#include <can_msgs/CanFrame.h>

//using namespace nhk2018_main_ros;

#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
	T data;
	uint64_t i;
};

template <typename T>
void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
{
	_Encapsulator<T> _e;

	for(int i = 0; i < sizeof(T); i++)
	{
		_e.i = (_e.i << 8) | (uint64_t)(buf[i]);
	}

	data = _e.data;
}

template<typename T>
void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
{
	_Encapsulator<T> _e;
	_e.data = data;

	for(int i = sizeof(T); i > 0;)
	{
		i--;
		buf[i] = _e.i & 0xff;
		_e.i >>= 8;
	}
}

class CrCanNode
{
public:
	CrCanNode(void);

private:
	void baseCmdCallback(const std_msgs::UInt16::ConstPtr& msg);
	void baseMotorCmdVelCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

	void handCmdCallback(const std_msgs::UInt16::ConstPtr& msg);

	void canRxCallback(const can_msgs::CanFrame::ConstPtr &msg);

	template<typename T>
	void sendData(const uint16_t id, const T data);

	ros::NodeHandle _nh;
	ros::Publisher _can_tx_pub;
	ros::Subscriber _can_rx_sub;

	ros::Publisher  _hand_status_pub;
	ros::Subscriber _hand_cmd_sub;


	ros::Publisher	_base_status_pub;
	ros::Subscriber	_base_cmd_sub;
	ros::Subscriber _base_motor_cmd_vel_sub;
	ros::Publisher  _base_odom_x_pub;
	ros::Publisher  _base_odom_y_pub;
	ros::Publisher  _base_odom_yaw_pub;

	static constexpr uint16_t id_baseStatus       = 0x200;
	static constexpr uint16_t id_baseCmd          = 0x201;
	static constexpr uint16_t id_baseMotorCmdVel0 = 0x202;
	static constexpr uint16_t id_baseMotorCmdVel1 = 0x203;
	static constexpr uint16_t id_baseMotorCmdVel2 = 0x204;
	static constexpr uint16_t id_baseOdomX        = 0x205;
	static constexpr uint16_t id_baseOdomY        = 0x206;
	static constexpr uint16_t id_baseOdomYaw      = 0x207;

	static constexpr uint16_t id_handStatus	= 0x300;
	static constexpr uint16_t id_handCmd	= 0x301;
};

CrCanNode::CrCanNode(void)
{
	_can_tx_pub				= _nh.advertise<can_msgs::CanFrame>("can_tx", 10);
	_can_rx_sub				= _nh.subscribe<can_msgs::CanFrame>("can_rx", 10, &CrCanNode::canRxCallback, this);

	_hand_status_pub	= _nh.advertise<std_msgs::UInt16>("hand/status", 10);
	_hand_cmd_sub		= _nh.subscribe<std_msgs::UInt16>("hand/cmd", 10, &CrCanNode::handCmdCallback, this);

	_base_status_pub		= _nh.advertise<std_msgs::UInt16>("base/status", 10);
	_base_cmd_sub			= _nh.subscribe<std_msgs::UInt16>("base/cmd", 10 , &CrCanNode::baseCmdCallback, this);
	_base_motor_cmd_vel_sub	= _nh.subscribe<std_msgs::Int16MultiArray>("base/motor_cmd_vel", 10 , &CrCanNode::baseMotorCmdVelCallback, this);
	_base_odom_x_pub		= _nh.advertise<std_msgs::Float64>("base/odom/x", 10);
	_base_odom_y_pub		= _nh.advertise<std_msgs::Float64>("base/odom/y", 10);
	_base_odom_yaw_pub		= _nh.advertise<std_msgs::Float64>("base/odom/yaw", 10);
}


void CrCanNode::baseCmdCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	this->sendData(id_baseCmd, msg->data);
}

void CrCanNode::baseMotorCmdVelCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	this->sendData(id_baseMotorCmdVel0, msg->data[0]);
	this->sendData(id_baseMotorCmdVel1, msg->data[1]);
	this->sendData(id_baseMotorCmdVel2, msg->data[2]);
}

void CrCanNode::handCmdCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	this->sendData(id_handCmd, msg->data);
}

void CrCanNode::canRxCallback(const can_msgs::CanFrame::ConstPtr &msg)
{
	std_msgs::UInt16 _hand_status_msg;
	std_msgs::UInt16 _base_status_msg;
	std_msgs::Float64 _base_odom_x_msg;
	std_msgs::Float64 _base_odom_y_msg;
	std_msgs::Float64 _base_odom_yaw_msg;
	switch(msg->id)
	{
	case id_handStatus:
		can_unpack(msg->data, _hand_status_msg.data);
		_hand_status_pub.publish(_hand_status_msg);
		break;

	case id_baseStatus:
		can_unpack(msg->data, _base_status_msg.data);
		_base_status_pub.publish(_base_status_msg);
		break;

	case id_baseOdomX:
		can_unpack(msg->data, _base_odom_x_msg.data);
		_base_odom_x_pub.publish(_base_odom_x_msg);
		break;

	case id_baseOdomY:
		can_unpack(msg->data, _base_odom_y_msg.data);
		_base_odom_y_pub.publish(_base_odom_y_msg);
		break;

	case id_baseOdomYaw:
		can_unpack(msg->data, _base_odom_yaw_msg.data);
		_base_odom_yaw_pub.publish(_base_odom_yaw_msg);
		break;

	default:
		break;
	}
}

template<typename T>
void CrCanNode::sendData(const uint16_t id, const T data)
{
	can_msgs::CanFrame frame;
	frame.id = id;
	frame.is_rtr = false;
	frame.is_extended = false;
	frame.is_error = false;

	frame.dlc = sizeof(T);

	can_pack<T>(frame.data, data);

	_can_tx_pub.publish(frame);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "cr_can");
	ROS_INFO("cr_can node has started.");

	CrCanNode *trCanNode = new CrCanNode();

	ros::spin();
}
