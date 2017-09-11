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
 * @file basic_control_node.h
 * @author Cyril Jourdan
 * @date Mar 14, 2017
 * @version OSA 2.0.0
 * @brief Header file for the ROS class BasicControlNode.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 14, 2017
 */

#ifndef OSA_GUI_ROSNODE_BASIC_CONTROL_NODE_H
#define OSA_GUI_ROSNODE_BASIC_CONTROL_NODE_H

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QString>
#include <QStringListModel>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>

namespace osa_gui
{
namespace rosnode
{

/**
 * @brief This defines the basic control ROS node.
 */
class BasicControlNode : public QThread
{
Q_OBJECT

public:
	/** @brief Constructor. */
	BasicControlNode();

	/** @brief Destructor. */
	virtual ~BasicControlNode();

	/** @brief Initialize the ROS node. */
	bool init();

	/** @brief Run the ROS node. */
	void run();

	/** @brief Callback method for the motor data. */
	void motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr &data);

	//setters
	int setEnablePublish(bool state);

	//getters
	std::string getRobotName() const { return robot_name_; };
	std::string getRobotCANDevice() const { return robot_can_device_; };
	int getNumberEPOSBoards() const { return number_epos_boards_; };
	QList<std::string> getDOFNameList() const { return dof_name_list_; };
	QList<std::string> getDOFTypeList() const { return dof_type_list_; };
	QList<int> getDOFNodeIDList() const { return dof_node_id_list_; };
	QList<std::string> getDOFControllerList() const { return dof_controller_list_; };
	QList<std::string> getDOFMotorList() const { return dof_motor_list_; };
	QList<bool> getDOFInvertedList() const { return dof_inverted_list_; };
	QList<std::string> getDOFModeList() const { return dof_mode_list_; };
	QList<int> getDOFValueList() const { return dof_value_list_; };
	osa_msgs::MotorDataMultiArray getMotorDataArray() const { return motor_data_array_; };
	bool getEnablePublish() const { return enable_publish_; };

Q_SIGNALS:
	void motorDataReceived(osa_msgs::MotorDataMultiArray data_array);
	//void statusChanged(StatusArray);
	//void loggingUpdated();

	/** @brief Signal to shutdown the ROS node. */
	void rosShutdown();

public Q_SLOTS:
	void setMotorCmdArray(osa_msgs::MotorCmdMultiArray motor_cmd_array);
	void updateMotorCmdArray(osa_msgs::MotorCmd motor_cmd);

private:
	void resetMotorCmdMultiArray();

private:
	/**< ros parameters*/
	std::string robot_name_;
	std::string robot_can_device_;
	int number_epos_boards_;

	/**< TODO DRY Move this in a class */
	QList<std::string> dof_name_list_;
	QList<std::string> dof_type_list_;
	QList<int> dof_node_id_list_;
	QList<std::string> dof_controller_list_;
	QList<std::string> dof_motor_list_;
	QList<bool> dof_inverted_list_;
	QList<std::string> dof_mode_list_;
	QList<int> dof_value_list_;

	/**< other attributes*/
	osa_msgs::MotorDataMultiArray motor_data_array_;
	osa_msgs::MotorCmdMultiArray motor_cmd_array_;
	ros::Publisher pub_motor_cmd_array_;
	ros::Subscriber sub_motor_data_array_;
	bool enable_publish_;
};

} // namespace rosnode
} // namespace osa_gui

#endif // OSA_GUI_ROSNODE_BASIC_CONTROL_NODE_H
