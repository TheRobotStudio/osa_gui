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
 * @file qnode.hpp
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version 0.0.1
 * @brief Header file for the ROS class QNode
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

#ifndef osa_gui_PLOTNODE_HPP_
#define osa_gui_PLOTNODE_HPP_

/*! Includes */
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <osa_msgs/MotorDataMultiArray.h>

/*! Namespaces */
namespace osa_gui
{
	namespace rosnode
	{
		/*! Class */
		class PlotNode : public QThread
		{
			Q_OBJECT

			public:
				PlotNode();
				virtual ~PlotNode();
				bool init();
				void motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr &data);
				void run();

				//getters
				osa_msgs::MotorDataMultiArray getMotorData_ma(); //access to private variables

			Q_SIGNALS:
				void motorDataReceived();
				void rosShutdown();

			private:
				osa_msgs::MotorDataMultiArray m_motorData_ma;
				ros::Subscriber m_sub_motorDataArray;
		};
	}	// namespace rosnode
}  // namespace osa_gui

#endif /* osa_gui_ROBOTPLOTNODE_HPP_ */
