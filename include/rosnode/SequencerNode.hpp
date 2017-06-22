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
 * @file SequencerNode.hpp
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version OSA 2.0.0
 * @brief Header file for the ROS class SequencerNode.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 21, 2017
 */

#ifndef osa_gui_SEQUENCERNODE_HPP_
#define osa_gui_SEQUENCERNODE_HPP_

/*! Includes */
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include "robotDefines.h"

/*! Namespaces */
namespace osa_gui
{
	namespace rosnode
	{
		/*! Class */
		class SequencerNode : public QThread
		{
			Q_OBJECT

			public:
				SequencerNode();
				virtual ~SequencerNode();
				bool init();
				void run();
				void motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr &data);

				//setters
				int setMotorCmd_ma(osa_msgs::MotorCmdMultiArray motorCmd_ma);
				int setEnablePublish(bool state);
				int setPause(int msPause);

				//getters
				osa_msgs::MotorDataMultiArray 	getMotorData_ma() 	const { return m_motorData_ma; }; //access to private variables
				bool getEnablePublish() const { return m_enablePublish; };

			Q_SIGNALS:
				void motorDataReceived(osa_msgs::MotorDataMultiArray data_ma);
				void rosShutdown();

			public Q_SLOTS:
				void updateMotorCmd_ma(osa_msgs::MotorCmdMultiArray motorCmd_ma);

			private:
				void resetMotorCmdMultiArray();

			private:
				osa_msgs::MotorDataMultiArray m_motorData_ma;
				osa_msgs::MotorCmdMultiArray m_motorCmd_ma;
				ros::Publisher m_pub_motorCmdArray;
				ros::Subscriber m_sub_motorDataArray;
				bool m_enablePublish;
				int m_pause;
		};
	}	// namespace rosnode
}  // namespace osa_gui

#endif /* osa_gui_SEQUENCERNODE_HPP_ */
