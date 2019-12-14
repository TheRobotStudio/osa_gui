/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 * @file face_tracking_node.h
 * @author Cyril Jourdan
 * @date Apr 26, 2017
 * @version OSA 2.0.0
 * @brief Header file for the ROS class FaceTrackingNode.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Apr 26, 2017
 */

#ifndef OSA_GUI_ROSNODE_FACE_TRACKING_NODE_H
#define OSA_GUI_ROSNODE_FACE_TRACKING_NODE_H

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include "robot_defines.h"

namespace osa_gui
{
namespace rosnode
{

/**
 * @brief This defines the face tracking ROS node.
 */
class FaceTrackingNode : public QThread
{
Q_OBJECT

public:
	/** @brief Constructor. */
	FaceTrackingNode();

	/** @brief Destructor. */
	virtual ~FaceTrackingNode();

	/** @brief Initialize the ROS node. */
	bool init();

	/** @brief Run the ROS node. */
	void run();

	//void motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr &data);
	void headCoordsCallback(const std_msgs::Int16MultiArrayConstPtr &head_coords);

	//setters
	int setMotorCmd_ma(osa_msgs::MotorCmdMultiArray motor_cmd_array);
	int setHeadCoords_ma(std_msgs::Int16MultiArray head_coords_array);
	int setEnableNode(bool enable_node);

	//getters
	//osa_msgs::MotorDataMultiArray 	getMotorData_ma() 	const { return m_motorData_ma; }; //access to private variables
	std_msgs::Int16MultiArray getHeadCoordsArray() const { return head_coords_array_; };
	bool getEnableNode() const { return enable_node_; };

Q_SIGNALS:
	//void motorDataReceived(osa_msgs::MotorDataMultiArray data_ma);
	void sendMotorCmd_ma(osa_msgs::MotorCmdMultiArray motor_cmd_array);

	/** @brief Signal to shutdown the ROS node. */
	void rosShutdown();

//public Q_SLOTS:
	//void updateMotorCmd_ma(osa_msgs::MotorCmd motorCmd);
	//int setPause(int msPause);

private:
	void resetMotorCmdMultiArray();
	void setNeckBackLeftRightPosition(int left_pos, int right_pos);
	void addNeckBackLeftRightPosition(int left_pos, int right_pos);
	void setEyeballPitchYaw(int pitch, int yaw);
	void computeEyballPosition();
	void computeIrisPosition();
	void computeFrontHeadPosition();
	void computeHeadPosition();

private:
	//osa_msgs::MotorDataMultiArray m_motorData_ma;
	osa_msgs::MotorCmdMultiArray motor_cmd_array_;
	std_msgs::Int16MultiArray head_coords_array_;
	//ros::Publisher m_pub_motorCmdArray;
	//ros::Subscriber m_sub_motorDataArray;

	ros::Subscriber sub_head_coords_;
	int face_pos_center_relative_[2];

	bool face_detected_;// = false;
	bool head_has_moved_;// = false;
	int prev_eye_pitch_pos_;// = EYEBALL_PITCH_CENTER;
	int prev_eye_yaw_pos_;// = EYEBALL_YAW_CENTER;
	int previous_iris_;// = 0;

	int prev_neck_back_left_;// = -MIDLE_POSITION;
	int prev_neck_back_right_;// = MIDLE_POSITION;

	int curr_head_pitch_;// = MIDLE_POSITION; //TODO
	int curr_head_yaw_;// = MIDLE_POSITION; //TODO

	int head_detected_counter_;// = 0;
	int no_head_detected_counter_;// = 0;

	bool manual_mode_;// = true;
	bool enable_node_;
};

} // namespace rosnode
} // namespace osa_gui

#endif // OSA_GUI_ROSNODE_FACE_TRACKING_NODE_H
