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
 * @file json_motor_cmd_multi_array.h
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for class JSONMotorCmdMultiArray
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef OSA_GUI_COMMON_OSA_MSGS_JSON_JSON_MOTOR_CMD_MULTI_ARRAY_H
#define OSA_GUI_COMMON_OSA_MSGS_JSON_JSON_MOTOR_CMD_MULTI_ARRAY_H

#include <QString>
#include <QJsonObject>
#include <QList>
#include "json_motor_cmd.h"

namespace osa_gui
{
namespace common
{
namespace osa_msgs_json
{

/**
 * @brief This class represents the QJsonObject form of the MotorCmdMultiArray ROS custom message.
 * This allows serialization of the ROS message for purposes like saving in a file.
 */
class JSONMotorCmdMultiArray
{
public:
	/**
	 * @brief Constructor.
	 */
	JSONMotorCmdMultiArray();

	/**
	 * @brief Destructor.
	 */
	~JSONMotorCmdMultiArray();

	//setters
	int addJSONMotorCmd(JSONMotorCmd* ptr_json_motor_cmd);

	//getters
	QList<JSONMotorCmd*> getJSONMotorCmdList() const { return json_motor_cmd_list_; };

	/** @brief Read method for JSON serialization. */
	void read(const QJsonObject &json);

	/** @brief Write method for JSON serialization. */
	void write(QJsonObject &json)  const;

protected:
	QList<JSONMotorCmd*> json_motor_cmd_list_;
};

} // namespace osa_msgs_json
} // namespace common
} // namespace osa_gui

#endif // OSA_GUI_COMMON_OSA_MSGS_JSON_JSON_MOTOR_CMD_MULTI_ARRAY_H
