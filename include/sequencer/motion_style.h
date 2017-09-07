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
 * @file motion_style.h
 * @author Cyril Jourdan
 * @date Mar 29, 2017
 * @version 0.1.0
 * @brief Header file for class MotionStyle
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 29, 2016
 */

#ifndef OSA_GUI_SEQUENCER_MOTIONSTYLE_H
#define OSA_GUI_SEQUENCER_MOTIONSTYLE_H

#include <sequence_element.h>
#include <QJsonObject>

namespace osa_gui
{
namespace sequencer
{

/*! \class MotionStyle
*  \brief
*
*  This class is
*/
class MotionStyle : public SequenceElement
{
public:
	/** @brief Constructor. */
	MotionStyle();

	/** @brief Destructor. */
	~MotionStyle();

	//setters
	int setModeOfOperation(int8_t modeOfOperation);
	int setProfileAcceleration(uint32_t profileAcceleration);
	int setProfileDeceleration(uint32_t profileDeceleration);
	int setProfileVelocity(uint32_t profileVelocity);
	int setOutputCurrentLimit(uint16_t outputCurrentLimit);

	//getters
	int8_t getModeOfOperation() 		const { return mode_of_operation_; };
	uint32_t getProfileAcceleration() 	const { return profile_acceleration_; };
	uint32_t getProfileDeceleration() 	const { return profile_deceleration_; };
	uint32_t getProfileVelocity() 		const { return profile_velocity_; };
	uint16_t getOutputCurrentLimit() 	const { return output_current_limit_; };

	void playElement(rosnode::SequencerNode* sequencerNode);

	/** @brief Read method for JSON serialization. */
	void read(const QJsonObject &json);

	/** @brief Write method for JSON serialization. */
	void write(QJsonObject &json)  const;

protected:
	int8_t mode_of_operation_;
	uint32_t profile_acceleration_;
	uint32_t profile_deceleration_;
	uint32_t profile_velocity_;
	uint16_t output_current_limit_;
};

} // namespace sequencer
} // namespace osa_gui

#endif // OSA_GUI_SEQUENCER_MOTIONSTYLE_H
