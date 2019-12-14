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
 * @file json_motor_cmd.h
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for class JSONMotorCmd
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef OSA_GUI_COMMON_OSA_MSGS_JSON_JSON_MOTOR_CMD_H
#define OSA_GUI_COMMON_OSA_MSGS_JSON_JSON_MOTOR_CMD_H

#include <QJsonObject>
#include <stdint.h>

namespace osa_gui
{
namespace common
{
namespace osa_msgs_json
{

/**
 * @brief This class represents the QJsonObject form of the MotorCmd ROS custom message.
 * This allows serialization of the ROS message for purposes like saving in a file.
 */
class JSONMotorCmd
{
public:
	/**
	 * @brief Constructor.
	 */
	JSONMotorCmd();

	/**
	 * @brief Destructor.
	 */
	~JSONMotorCmd();
/*
	//setters
	int setNodeID(uint8_t node_id);
	int setCommand(uint8_t command);
	int setValue(int32_t value);

	//getters
	uint8_t getNodeID() const { return node_id_; };
	uint8_t getCommand() const { return command_; };
	int32_t getValue() const { return value_; };
*/
	/** @brief Read method for JSON serialization. */
	void read(const QJsonObject &json);

	/** @brief Write method for JSON serialization. */
	void write(QJsonObject &json)  const;

protected:
	uint8_t node_id_;
	uint8_t command_;
	int32_t value_;
};

} // namespace osa_msgs_json
} // namespace common
} // namespace osa_gui

#endif // OSA_GUI_COMMON_OSA_MSGS_JSON_JSON_MOTOR_CMD_H
