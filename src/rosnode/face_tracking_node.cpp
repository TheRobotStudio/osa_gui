/*
 * Copyright (c) 2017, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file face_tracking_node.cpp
 * @author Cyril Jourdan
 * @date Apr 26, 2017
 * @version OSA 2.0.0
 * @brief Implementation file for the ROS class FaceTrackingNode.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Apr 26, 2017
 */

/*! Includes */
#include <face_tracking_node.h>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "robot_defines.h"

/*! Defines */
#define LOOP_RATE				HEART_BEAT

#define HEAD_NECK_FRONT_RIGHT_ID 	1
#define HEAD_NECK_FRONT_LEFT_ID 	2
#define HEAD_NECK_BACK_RIGHT_ID 	3
#define HEAD_NECK_BACK_LEFT_ID 		4
#define HEAD_EYE_PITCH_ID			5
#define HEAD_EYE_YAW_ID				6
#define HEAD_EYE_IRIS_ID			7

/*! Namespaces */
using namespace osa_gui;
using namespace rosnode;

FaceTrackingNode::FaceTrackingNode()// :
/*	//m_motorCmdSet,
	//face_pos_center_relative_[2],
	face_detected_(false),
	head_has_moved_(false),
	prev_eye_pitch_pos_(EYEBALL_PITCH_CENTER),
	prev_eye_yaw_pos_(EYEBALL_YAW_CENTER)
	previous_iris_(0),
	prev_neck_back_left_(-MIDLE_POSITION),
	prev_neck_back_right_(MIDLE_POSITION),
	curr_head_pitch_(MIDLE_POSITION),
	curr_head_yaw_(MIDLE_POSITION),
	head_detected_counter_(0),
	no_head_detected_counter_(0),
	manual_mode_(true)*/
{
	//m_motorCmdSet;
	//face_pos_center_relative_[2];
	face_detected_ = false;
	head_has_moved_ = false;

	//previous values of the eyeball
	prev_eye_pitch_pos_ = EYEBALL_PITCH_CENTER;
	prev_eye_yaw_pos_ = EYEBALL_YAW_CENTER;
	previous_iris_ = 0;

	prev_neck_back_left_ = MIDLE_POSITION;
	prev_neck_back_right_ = MIDLE_POSITION;

	curr_head_pitch_ = MIDLE_POSITION; //TODO
	curr_head_yaw_ = MIDLE_POSITION; //TODO

	head_detected_counter_ = 0;
	no_head_detected_counter_ = 0;

	manual_mode_ = true;
	enable_node_ = false;
}

FaceTrackingNode::~FaceTrackingNode()
{
	//delete m_motorData_ma;

	if(ros::isStarted())
	{
	  ros::shutdown(); // explicitly needed since we use ros::start();
	  ros::waitForShutdown();
	}
	wait();
}

bool FaceTrackingNode::init()
{
	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_gui_FaceTrackingNode");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh;

	//Publishers
	//m_pub_motorCmdArray = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//Subscribers
	//m_sub_motorDataArray = nh.subscribe("/motor_data_array", 10, &FaceTrackingNode::motorDataArray_cb, this);
	sub_head_coords_ = nh.subscribe("/headCoords", 10, &FaceTrackingNode::headCoordsCallback, this);

	//create the commands multi array
	//Assuming the head is using a SlaveBoard by itself
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = 1;
	motor_cmd_array_.layout.dim[0].stride = 1*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "slave";

	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].label = "motors";

	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	resetMotorCmdMultiArray();

	//Initialize the headCoords
	head_coords_array_.data.clear();
	head_coords_array_.data.push_back(-1); //face X
	head_coords_array_.data.push_back(-1); //face Y
	head_coords_array_.data.push_back(-1); //face size
	head_coords_array_.data.push_back(-1); //face size

	face_pos_center_relative_[0] = 0;
	face_pos_center_relative_[1] = 0;

	start();

	return true;
}

void FaceTrackingNode::run()
{
	ros::Rate r(LOOP_RATE);

	while(ros::ok())
	{
		if(enable_node_)
		{
			ros::spinOnce();

			if(face_detected_)
			{
				//call function that will compute new position values for the head
				computeHeadPosition();
			}
			else
			{
				//re-center head after 2 seconds without face detected
				if(no_head_detected_counter_ > NO_HEAD_DETECTED_COUNTER)
				{
					//ROS_INFO("Recenter Head");
					//set eyeball to center position
					int eyeP = EYEBALL_PITCH_CENTER;
					int eyeY = EYEBALL_YAW_CENTER;
					setEyeballPitchYaw(eyeP, eyeY);
					//set iris position
					computeIrisPosition();
					//set neck to center position
					int leftP = MIDLE_POSITION;
					int rightP = MIDLE_POSITION;
					setNeckBackLeftRightPosition(leftP, rightP);
					no_head_detected_counter_ = 0;
				}
			}

			//TODO make the command builder fast and dynamic

			//EMIT the updated command
			Q_EMIT sendMotorCmd_ma(motor_cmd_array_);

			//ros::Duration(5).sleep();

			resetMotorCmdMultiArray();

			//r.sleep();
		}

		r.sleep();
	}

	//std::cout << "FaceTrackingNode (ROS node) shutdown, proceeding to close the GUI." << std::endl;
	ROS_INFO("FaceTrackingNode (ROS node) shutdown, proceeding to close the GUI.");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

int FaceTrackingNode::setEnableNode(bool enableNode)
{
	enable_node_  = enableNode;

	return 0;
}

//Functions
void FaceTrackingNode::setNeckBackLeftRightPosition(int leftPos, int rightPos)
{
	int neckBackLeftPos = leftPos;
	int neckBackRightPos = rightPos;

	//clip
	if(neckBackLeftPos < DCX_NECK_BACK_LEFT_MIN) neckBackLeftPos = DCX_NECK_BACK_LEFT_MIN;
	if(neckBackLeftPos > DCX_NECK_BACK_LEFT_MAX) neckBackLeftPos = DCX_NECK_BACK_LEFT_MAX;
	if(neckBackRightPos < DCX_NECK_BACK_RIGHT_MIN) neckBackRightPos = DCX_NECK_BACK_RIGHT_MIN;
	if(neckBackRightPos > DCX_NECK_BACK_RIGHT_MAX) neckBackRightPos = DCX_NECK_BACK_RIGHT_MAX;

	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_LEFT_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_LEFT_ID-1].value = neckBackLeftPos;
	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_RIGHT_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_RIGHT_ID-1].value = neckBackRightPos;

	prev_neck_back_left_ = neckBackLeftPos;
	prev_neck_back_right_ = neckBackRightPos;
}

void FaceTrackingNode::addNeckBackLeftRightPosition(int leftPos, int rightPos)
{
	int neckBackLeftPos = prev_neck_back_left_ + leftPos;
	int neckBackRightPos = prev_neck_back_right_ + rightPos;

	//clip
	if(neckBackLeftPos < DCX_NECK_BACK_LEFT_MIN) neckBackLeftPos = DCX_NECK_BACK_LEFT_MIN;
	if(neckBackLeftPos > DCX_NECK_BACK_LEFT_MAX) neckBackLeftPos = DCX_NECK_BACK_LEFT_MAX;
	if(neckBackRightPos < DCX_NECK_BACK_RIGHT_MIN) neckBackRightPos = DCX_NECK_BACK_RIGHT_MIN;
	if(neckBackRightPos > DCX_NECK_BACK_RIGHT_MAX) neckBackRightPos = DCX_NECK_BACK_RIGHT_MAX;

	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_LEFT_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_LEFT_ID-1].value = neckBackLeftPos;
	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_RIGHT_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_NECK_BACK_RIGHT_ID-1].value = neckBackRightPos;

	prev_neck_back_left_ = neckBackLeftPos;
	prev_neck_back_right_ = neckBackRightPos;
}

void FaceTrackingNode::setEyeballPitchYaw(int pitch, int yaw)
{
	int eyePitchPos = pitch;
	int eyeYawPos = yaw;

	//clip
	if(eyePitchPos < DCX_EYEBALL_PITCH_MIN) eyePitchPos = DCX_EYEBALL_PITCH_MIN;
	if(eyePitchPos > DCX_EYEBALL_PITCH_MAX) eyePitchPos = DCX_EYEBALL_PITCH_MAX;
	if(eyeYawPos < DCX_EYEBALL_YAW_MIN) eyeYawPos = DCX_EYEBALL_YAW_MIN;
	if(eyeYawPos > DCX_EYEBALL_YAW_MAX) eyeYawPos = DCX_EYEBALL_YAW_MAX;

	//call the eyeball commands
	//pitch
	motor_cmd_array_.motor_cmd[HEAD_EYE_PITCH_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_EYE_PITCH_ID-1].value = eyePitchPos;
	//yaw
	motor_cmd_array_.motor_cmd[HEAD_EYE_YAW_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_EYE_YAW_ID-1].value = eyeYawPos;

	//ROS_INFO("PY[%d\t%d]", motorCmdSet.motor_cmd[HEAD_EYE_PITCH_ID-1].value, motorCmdSet.motor_cmd[HEAD_EYE_YAW_ID-1].value);

	prev_eye_pitch_pos_ = eyePitchPos;
	prev_eye_yaw_pos_ = eyeYawPos;
}

void FaceTrackingNode::computeEyballPosition()
{
	int eyePitchPos;
	int eyeYawPos;
	int x = face_pos_center_relative_[0];
	int y = face_pos_center_relative_[1];

	//ROS_INFO("facePosCenterRelative[%d, %d]", x, y);

	if(x > 0)
	{
		eyeYawPos = prev_eye_yaw_pos_ - x*EYE_SPEED_FACTOR; //TODO

		if(eyeYawPos < DCX_EYEBALL_YAW_MIN)
		{
			eyeYawPos = DCX_EYEBALL_YAW_MIN;
			//turnHeadLeft(3); //TODO
			ROS_INFO("turnHeadLeft");
			//addNeckBackLeftRightPosition(-500, -500);
			addNeckBackLeftRightPosition(500, -500);
		}
	}
	else
	{
		eyeYawPos = prev_eye_yaw_pos_ - x*EYE_SPEED_FACTOR; //TODO

		if(eyeYawPos > DCX_EYEBALL_YAW_MAX)
		{
			eyeYawPos = DCX_EYEBALL_YAW_MAX;
			//turnHeadRight(3); //TODO
			ROS_INFO("turnHeadRight");
			addNeckBackLeftRightPosition(-500, 500);
		}
	}

	if(y > 0)
	{
		eyePitchPos = prev_eye_pitch_pos_ - y*EYE_SPEED_FACTOR; //TODO

		if(eyePitchPos < DCX_EYEBALL_PITCH_MIN)
		{
			eyePitchPos = DCX_EYEBALL_PITCH_MIN;
			//turnHeadDown(3); //TODO
			ROS_INFO("turnHeadDown");
			addNeckBackLeftRightPosition(-500, -500);
		}
	}
	else
	{
		eyePitchPos = prev_eye_pitch_pos_ - y*EYE_SPEED_FACTOR; //TODO

		if(eyePitchPos > DCX_EYEBALL_PITCH_MAX)
		{
			eyePitchPos = DCX_EYEBALL_PITCH_MAX;
			//turnHeadUp(3); //TODO
			ROS_INFO("turnHeadUp");
			addNeckBackLeftRightPosition(500, 500);
		}
	}

	//clip
	if(eyePitchPos < DCX_EYEBALL_PITCH_MIN) eyePitchPos = DCX_EYEBALL_PITCH_MIN;
	if(eyePitchPos > DCX_EYEBALL_PITCH_MAX) eyePitchPos = DCX_EYEBALL_PITCH_MAX;
	if(eyeYawPos < DCX_EYEBALL_YAW_MIN) eyeYawPos = DCX_EYEBALL_YAW_MIN;
	if(eyeYawPos > DCX_EYEBALL_YAW_MAX) eyeYawPos = DCX_EYEBALL_YAW_MAX;

	//call the eyeball commands
	//pitch
	motor_cmd_array_.motor_cmd[HEAD_EYE_PITCH_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_EYE_PITCH_ID-1].value = eyePitchPos;
	//yaw
	motor_cmd_array_.motor_cmd[HEAD_EYE_YAW_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_EYE_YAW_ID-1].value = eyeYawPos;

	//ROS_INFO("PY[%d\t%d]", motorCmdSet.motor_cmd[HEAD_EYE_PITCH_ID-1].value, motorCmdSet.motor_cmd[HEAD_EYE_YAW_ID-1].value);

	prev_eye_pitch_pos_ = eyePitchPos;
	prev_eye_yaw_pos_ = eyeYawPos;
}

void FaceTrackingNode::computeIrisPosition()
{
	int iris_pos = 0;
	int generator = rand() % 500;
	int irisRandPos = generator - 250;

	//if stare at someone for a long time
	if(head_detected_counter_ > HEAD_DETECTED_COUNTER)
	{
		//ROS_INFO("FOCUS");
		iris_pos = IRIS_FOCUS_POSITION + 2*irisRandPos;
	}

	if(no_head_detected_counter_ > NO_HEAD_DETECTED_COUNTER)
	{
		//ROS_INFO("AWARE");
		iris_pos = IRIS_AWARE_POSITION + 2*irisRandPos;
	}

	//random movements
	if((head_detected_counter_ < HEAD_DETECTED_COUNTER) && (no_head_detected_counter_ < NO_HEAD_DETECTED_COUNTER))
	{
		//move the iris randomly
		//iris_pos = previousIris + irisRandPos;
		iris_pos = 8000 + 2*irisRandPos;
	}

	//clip
	if(iris_pos < DCX_EYEBALL_IRIS_MIN) iris_pos = DCX_EYEBALL_IRIS_MIN;
	if(iris_pos > DCX_EYEBALL_IRIS_MAX) iris_pos = DCX_EYEBALL_IRIS_MAX;

	//update iris position
	motor_cmd_array_.motor_cmd[HEAD_EYE_IRIS_ID-1].command = SET_TARGET_POSITION;
	motor_cmd_array_.motor_cmd[HEAD_EYE_IRIS_ID-1].value = iris_pos;

	//ROS_INFO("iris_pos[%d]", motorCmdSet.motor_cmd[HEAD_EYE_IRIS_ID-1].value);

	//save previous iris position
	previous_iris_ = iris_pos;
}

void FaceTrackingNode::computeFrontHeadPosition()
{
	motor_cmd_array_.motor_cmd[HEAD_NECK_FRONT_LEFT_ID-1].command = SET_CURRENT_MODE_SETTING_VALUE;
	motor_cmd_array_.motor_cmd[HEAD_NECK_FRONT_LEFT_ID-1].value = DCX_NECK_FRONT_LEFT_CURR;
	motor_cmd_array_.motor_cmd[HEAD_NECK_FRONT_RIGHT_ID-1].command = SET_CURRENT_MODE_SETTING_VALUE;
	motor_cmd_array_.motor_cmd[HEAD_NECK_FRONT_RIGHT_ID-1].value = DCX_NECK_FRONT_RIGHT_CURR;
}

void FaceTrackingNode::computeHeadPosition()
{
	//update the eyeball pitch and yaw
	computeEyballPosition();
	//update the iris
	computeIrisPosition();
	//update the front head motors
	computeFrontHeadPosition();

	//ROS_INFO("Head = EyeBall(pitch %d, yaw %d, roll %d, iris %d) - Neck(LB %d, RB %d, LM %d, RM %d)", motorCmdSet.motor_cmd[HEAD_EYE_PITCH_ID-1].value, motorCmdSet.motor_cmd[YAW_ID-1].value, motorCmdSet.motor_cmd[ROLL_ID-1].value, motorCmdSet.motor_cmd[HEAD_EYE_IRIS_ID-1].value, motorCmdSet.motor_cmd[HEAD_NECK7_ID-1].value, motorCmdSet.motor_cmd[HEAD_NECK8_ID-1].value, motorCmdSet.motor_cmd[HEAD_NECK9_ID-1].value, motorCmdSet.motor_cmd[HEAD_NECK10_ID-1].value);
}

//callbacks
/*
void FaceTrackingNode::motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	m_motorData_ma = *data;
	Q_EMIT motorDataReceived(m_motorData_ma);
}*/

void FaceTrackingNode::headCoordsCallback(const std_msgs::Int16MultiArrayConstPtr &head_coords)
{
	head_coords_array_ = *head_coords;

	if((head_coords->data[0] != -1) && (head_coords->data[1] != -1)) //if head detected
	{
		//update head coords using 640*480 image
		face_pos_center_relative_[0] = 320-head_coords->data[0];
		face_pos_center_relative_[1] = head_coords->data[1]-240; //240 - headCoords->data[1];

		face_detected_ = true;
		head_detected_counter_++;
		no_head_detected_counter_ = 0;
	}
	else
	{
		face_detected_ = false;
		head_detected_counter_ = 0;
		no_head_detected_counter_ ++;
	}

	//compute head coordinates
	//m_motorData_ma

	//ROS_INFO("HEAD[%d, %d, %d, %d]", head_coords_array_.data[0], head_coords_array_.data[1], head_coords_array_.data[2], head_coords_array_.data[3]);
}

//setters
int FaceTrackingNode::setMotorCmd_ma(osa_msgs::MotorCmdMultiArray motorCmd_ma)
{
	motor_cmd_array_ = motorCmd_ma;

	return 0;
}

int FaceTrackingNode::setHeadCoords_ma(std_msgs::Int16MultiArray headCoords_ma)
{
	head_coords_array_ = headCoords_ma;

	return 0;
}

void FaceTrackingNode::resetMotorCmdMultiArray()
{
	for(int i=0; i<1; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			//motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].node_id = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SEND_DUMB_MESSAGE;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0;
		}
	}
}
