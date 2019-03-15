/*
 * tr_thrower_test.cpp
 *
 *  Created on: Feb 14, 2019
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <string>

#include "Coordinates.hpp"

class MR2LegUnitTest
{
public:
    MR2LegUnitTest(void);

    static constexpr double timer_inc = 0.005;

private:

    void control_timer_callback(const ros::TimerEvent& event);
    void motor0_stat_callback(const std_msgs::UInt8::ConstPtr& msg);
    void motor1_stat_callback(const std_msgs::UInt8::ConstPtr& msg);
    void motor2_stat_callback(const std_msgs::UInt8::ConstPtr& msg);

    ros::NodeHandle nh_;

    ros::Publisher motor0_cmd_pub;
    std_msgs::UInt8 motor0_cmd_msg;
    ros::Publisher motor1_cmd_pub;
    std_msgs::UInt8 motor1_cmd_msg;
    ros::Publisher motor2_cmd_pub;
    std_msgs::UInt8 motor2_cmd_msg;

    ros::Publisher motor0_cmd_pos_pub;
    std_msgs::Float32 motor0_cmd_pos_msg;
    ros::Publisher motor1_cmd_pos_pub;
    std_msgs::Float32 motor1_cmd_pos_msg;
    ros::Publisher motor2_cmd_pos_pub;
    std_msgs::Float32 motor2_cmd_pos_msg;

    ros::Subscriber motor0_stat_sub;
    ros::Subscriber motor1_stat_sub;
    ros::Subscriber motor2_stat_sub;

    ros::Timer control_timer;

    std::vector<double> motor0_traj;
    std::vector<double> motor1_traj;
    std::vector<double> motor2_traj;

    bool _motor0_homing_cplt = false;
    bool _motor1_homing_cplt = false;

    bool _motor0_stat_prev = 0x00;
    bool _motor1_stat_prev = 0x00;
};

const double l1 = 0.2075;
const double l2 = 0.246;

void inv_kinematic(double x, double y, double &theta1, double &theta2)
{
    auto x_sq = pow(x, 2);
    auto y_sq = pow(y, 2);
    auto l1_sq = pow(l1, 2);
    auto l2_sq = pow(l2, 2);
    double d1 = (x_sq + y_sq + l1_sq + l2_sq) / (2 * l1);
    double d2 = (x_sq + y_sq - l1_sq + l2_sq) / (2 * l2);
    auto d1_sq = pow(d1, 2);
    auto d2_sq = pow(d2, 2);
    theta1 = atan2(y, x) + atan2(sqrt(x_sq + y_sq - d1_sq), d1);
    theta2 = -atan2(sqrt(x_sq + y_sq - d1_sq), d1) - atan2(sqrt(x_sq + y_sq - d2_sq), d2);

    theta1 = M_PI - theta1;
    theta2 = M_PI - theta2;
}

void calc_traj(std::vector<double> &traj, double x_0, double x_f, double v_0, double v_f)
{
    //traj.clear();

    double a = 2 * (x_0 - x_f) + v_0 + v_f;
    double b = 3 * (x_f - x_0) - 2 * v_0 - v_f;
    double c = v_0;
    double d = x_0;
    double t = 0.0;

    double k = 1.75;

    // one per 10 ms
    for (int i = 0; i < 1 / (MR2LegUnitTest::timer_inc / k); i++)
    {
        double x = a * pow(t, 3) + b * pow(t, 2) + c * t + d;
        traj.push_back(x);
        t += (MR2LegUnitTest::timer_inc / k);
    }
}

MR2LegUnitTest::MR2LegUnitTest(void)
{
    this->motor0_cmd_pub = nh_.advertise<std_msgs::UInt8>("base/motor0_cmd", 10);
    this->motor1_cmd_pub = nh_.advertise<std_msgs::UInt8>("base/motor1_cmd", 10);
    this->motor2_cmd_pub = nh_.advertise<std_msgs::UInt8>("base/motor2_cmd", 10);
    this->motor0_cmd_pos_pub = nh_.advertise<std_msgs::Float32>("base/motor0_cmd_vel", 10);
    this->motor1_cmd_pos_pub = nh_.advertise<std_msgs::Float32>("base/motor1_cmd_vel", 10);
    this->motor2_cmd_pos_pub = nh_.advertise<std_msgs::Float32>("base/motor2_cmd_vel", 10);
    this->motor0_stat_sub = nh_.subscribe("base/motor0_stat", 10, &MR2LegUnitTest::motor0_stat_callback, this);
    this->motor1_stat_sub = nh_.subscribe("base/motor1_stat", 10, &MR2LegUnitTest::motor1_stat_callback, this);
    //this->motor2_stat_sub = nh_.subscribe("base/motor2_stat", 10, &MR2LegUnitTest::motor2_stat_callback, this);

    // timer starts immediately
    control_timer = nh_.createTimer(ros::Duration(timer_inc), &MR2LegUnitTest::control_timer_callback, this);

    //double x_00 = -M_PI/2;
    //double x_01 = -2.7 * M_PI/4;
    //calc_traj(motor0_traj, x_00, M_PI/2, 0, 0);
    //calc_traj(motor1_traj, x_01, M_PI / 4, 0, 0);
    //for(int i = 0; i < 100; i++)
    //{
    //    motor0_traj.insert(motor0_traj.begin(), x_00);
    //    motor1_traj.insert(motor1_traj.begin(), x_01);
    //}

    //calc_traj(motor0_traj, 0, -M_PI / 2, 0, 0);
    //calc_traj(motor1_traj, 0, -2.7 * M_PI / 4, 0, 0);

    std::vector<double> x_traj;
    std::vector<double> y_traj;

    motor0_traj.clear();
    motor1_traj.clear();
    motor2_traj.clear();

    //calc_traj(x_traj, 0, 0.1, 0, 0);
    //calc_traj(y_traj, 0, 0.1, 0, 0);

    //for(auto &)

    calc_traj(motor0_traj, 0, -M_PI / 2, 0, 0);
    calc_traj(motor1_traj, 0, -M_PI / 2, 0, 0);

    calc_traj(motor0_traj, M_PI, M_PI / 2, 0, 0);
    calc_traj(motor1_traj, M_PI, M_PI, 0, 0);

    calc_traj(motor0_traj, M_PI / 2, 0, 0, 0);
    calc_traj(motor1_traj, M_PI, 0, 0, 0);

    /*
     calc_traj(motor0_traj, 0, M_PI, 0, 0);
     calc_traj(motor1_traj, 0, M_PI, 0, 0);
     calc_traj(motor2_traj, 0, -M_PI / 2, 0, 0);

     calc_traj(motor0_traj, M_PI, M_PI / 2, 0, 0);
     calc_traj(motor1_traj, M_PI, M_PI, 0, 0);
     calc_traj(motor2_traj, -M_PI / 2, -M_PI, 0, 0);

     calc_traj(motor0_traj, M_PI / 2, 0, 0, 0);
     calc_traj(motor1_traj, M_PI, 0, 0, 0);
     calc_traj(motor2_traj, -M_PI, 0, 0, 0);
     */

}

void MR2LegUnitTest::control_timer_callback(const ros::TimerEvent& event)
{
    static int i = 0;
    static bool _first = true;
    static bool _homing_cplt = false;

    static bool _end = false;

    if (!_homing_cplt)
    {
        if (!this->_motor0_homing_cplt)
        {
            if (_first)
            {
                if (motor0_cmd_pub.getNumSubscribers() == 0 || motor1_cmd_pub.getNumSubscribers() == 0)
                {
                    return;
                }

                ROS_INFO("performing homing process...");
                motor0_cmd_msg.data = 0x10;
                motor0_cmd_pub.publish(motor0_cmd_msg);

                _first = false;
            }
            else if (this->_motor0_homing_cplt)
            {
                ROS_INFO("one done, one to go");
                _first = true;
            }
        }
        else if (!this->_motor1_homing_cplt)
        {
            if (_first)
            {
                motor1_cmd_msg.data = 0x10;
                motor1_cmd_pub.publish(motor1_cmd_msg);

                _first = false;
            }
            else if (this->_motor1_homing_cplt)
            {
                ROS_INFO("homing cplt.");
                _first = true;

                motor0_cmd_msg.data = 0x00;
                motor0_cmd_pub.publish(motor0_cmd_msg);
                motor1_cmd_msg.data = 0x00;
                motor1_cmd_pub.publish(motor1_cmd_msg);

            }
        }
    }

    return;

    if (_first)
    {
        ROS_INFO("restarting...");
        motor0_cmd_msg.data = 0x0001;
        motor0_cmd_pub.publish(motor0_cmd_msg);
        motor1_cmd_msg.data = 0x0001;
        motor1_cmd_pub.publish(motor1_cmd_msg);
        motor2_cmd_msg.data = 0x0001;
        motor2_cmd_pub.publish(motor2_cmd_msg);

        _first = false;
        return;
    }

    if (i < motor0_traj.size() && i < motor1_traj.size())
    {
        motor0_cmd_pos_msg.data = motor0_traj.at(i);
        motor0_cmd_pos_pub.publish(motor0_cmd_pos_msg);

        motor1_cmd_pos_msg.data = motor1_traj.at(i);
        motor1_cmd_pos_pub.publish(motor1_cmd_pos_msg);

        motor2_cmd_pos_msg.data = motor2_traj.at(i);
        motor2_cmd_pos_pub.publish(motor2_cmd_pos_msg);

        i++;

        ROS_INFO("sending: step %d", i);
    }
    else if (!_end)
    {
        ROS_INFO("motion cplt.");

        //motor0_cmd_msg.data = 0x0000;
        //motor0_cmd_pub.publish(motor0_cmd_msg);
        //motor1_cmd_msg.data = 0x0000;
        //motor1_cmd_pub.publish(motor1_cmd_msg);

        _end = true;
    }
}

void MR2LegUnitTest::motor0_stat_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    if (msg->data != 0x10 && this->_motor0_stat_prev == 0x10)
    {
        this->_motor0_homing_cplt = true;
    }

    if (msg->data == 0x10 && this->_motor0_stat_prev != 0x10)
    {
        this->_motor0_homing_cplt = false;
    }

    this->_motor0_stat_prev = msg->data;
}

void MR2LegUnitTest::motor1_stat_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    if (msg->data != 0x10 && this->_motor1_stat_prev == 0x10)
    {
        this->_motor1_homing_cplt = true;
    }

    if (msg->data == 0x10 && this->_motor1_stat_prev != 0x10)
    {
        this->_motor1_homing_cplt = false;
    }

    this->_motor1_stat_prev = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tr_main");

    MR2LegUnitTest *instance = new MR2LegUnitTest();
    ROS_INFO("MR2 unit test node has started.");

    ros::spin();
    ROS_INFO("MR2 unit test node has been terminated.");
}

