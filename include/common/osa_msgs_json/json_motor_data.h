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
 * @file json_motor_data.h
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for class JSONMotorData
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef JSONMOTORDATA_H_
#define JSONMOTORDATA_H_

#include <QJsonObject>
#include <stdint.h>

namespace osa_gui
{
namespace common
{
namespace osa_msgs_json
{

/**
 * @brief This class represents the QJsonObject form of the MotorDataMultiArray ROS custom message.
 * This allows serialization of the ROS message for purposes like saving in a file.
 */
class JSONMotorData
{
public:
	/** @brief Constructor. */
	JSONMotorData();

	/** @brief Destructor. */
	~JSONMotorData();

	int setPosition(int32_t position);
	int setCurrent(int16_t current);
	int setStatus(uint16_t status);

	int32_t getPosition() const { return position_; };
	int16_t getCurrent() const { return current_; };
	uint16_t getStatus() const { return status_; };

	/** @brief Read method for JSON serialization. */
	void read(const QJsonObject &json);

	/** @brief Write method for JSON serialization. */
	void write(QJsonObject &json)  const;

protected:
	int32_t position_;
	int32_t velocity_;
	int16_t current_;
	int16_t following_error_;
	uint16_t status_;
	int8_t mode_of_operation_;
};

} // namespace osa_msgs_json
} // namespace common
} // namespace osa_gui

#endif /* JSONMOTORDATA_H_ */
