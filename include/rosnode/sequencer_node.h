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
 * @file sequencer_node.h
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version OSA 2.0.0
 * @brief Header file for the ROS class SequencerNode.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 21, 2017
 */

#ifndef OSA_GUI_ROSNODE_SEQUENCER_NODE_H
#define OSA_GUI_ROSNODE_SEQUENCER_NODE_H

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
 * @brief This defines the sequencer ROS node.
 */
class SequencerNode : public QThread
{
Q_OBJECT

public:
	/** @brief Constructor. */
	SequencerNode();

	/** @brief Destructor. */
	virtual ~SequencerNode();

	/** @brief Initialize the ROS node. */
	bool init();

	/** @brief Run the ROS node. */
	void run();

	/** @brief Callback method for the motor data. */
	void motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr &data);

	//setters
	int setMotorCmdArray(osa_msgs::MotorCmdMultiArray motor_cmd_array);
	int setEnablePublish(bool state);

	/**
	 * @brief Run the ROS node.
	 * @param pause Time in ms.
	 * @return int
	 */
	int setPause(int pause);

	//getters
	osa_msgs::MotorDataMultiArray getMotorDataArray() const { return motor_data_array_; }; /**< Access to private variables. */
	bool getEnablePublish() const { return enable_publish_; };

Q_SIGNALS:
	void motorDataReceived(osa_msgs::MotorDataMultiArray data_array);

	/** @brief Signal to shutdown the ROS node. */
	void rosShutdown();

public Q_SLOTS:
	void updateMotorCmdArray(osa_msgs::MotorCmdMultiArray motor_cmd_array);

private:
	void resetMotorCmdMultiArray();

private:
	osa_msgs::MotorDataMultiArray motor_data_array_;
	osa_msgs::MotorCmdMultiArray motor_cmd_array_;
	ros::Publisher pub_motor_cmd_array_;
	ros::Subscriber sub_motor_data_array_;
	bool enable_publish_;
	int pause_; /**< Pause in ms. */
};
} // namespace rosnode
} // namespace osa_gui

#endif // OSA_GUI_ROSNODE_SEQUENCER_NODE_H
