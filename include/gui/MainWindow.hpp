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
 * @file MainWindow.hpp
 * @author Cyril Jourdan
 * @date Mar 15, 2017
 * @version OSA 2.0.0
 * @brief Header file for Qt based gui class MainWindow.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

#ifndef osa_gui_MAIN_WINDOW_H
#define osa_gui_MAIN_WINDOW_H

/*! Includes */

#include <QFile>
#include <include/gui/ConfigurationGUI.hpp>
#include <include/gui/BasicControlGUI.hpp>
#include <include/gui/SequencerGUI.hpp>
#include <include/gui/PlotGUI.hpp>
#include <include/gui/SupervisorGUI.hpp>
#include <include/common/Robot.hpp>
#include <Project.hpp>
#include <QMainWindow>
#include "ui_MainWindow.h"

/*! Namespace */
namespace osa_gui
{
	namespace gui
	{
		//typedef QList<int> StatusArray;

		/*! Interface [MainWindow] */
		/**
		 * @brief Qt central, all operations relating to the view part here.
		 */
		class MainWindow : public QMainWindow
		{
			Q_OBJECT

			public:
				MainWindow(int argc, char** argv, QWidget *parent = 0);
				~MainWindow();

				//To read and write settings and preferences. (just use for window geometry now, other things done with JSON)
				void ReadSettings(); // Load up Qt program settings at startup
				void WriteSettings(); // Save Qt program settings when closing

				//To read and write objects with JSON
				void read(const QJsonObject &json);
				void write(QJsonObject &json) const;

				void closeEvent(QCloseEvent *event); //Overloaded function

			public Q_SLOTS:
				/*! Auto-connections (connectSlotsByName()) */
				//Menu
				void on_actionAbout_triggered();
				//Toobbar actions
				void on_actionNewProject_triggered();
				void on_actionOpenProject_triggered();
				void on_actionSaveProject_triggered();
				void on_actionSaveProjectAs_triggered();
				void on_actionOpenTerminal_triggered();

				void on_actionToolBar_triggered();
				void on_actionStatusBar_triggered();
				//Robot
				void on_actionOpenConfiguration_triggered();
				void on_actionOpenControl_triggered();
				void on_actionOpenSequencer_triggered();
				void on_actionOpenPlot_triggered();
				void on_actionOpenSupervisor_triggered();

			private:
				Ui::MainWindowDesign 	m_ui;
				ConfigurationGUI* 		m_configurationGui; //TODO add p for pointer in the names
				BasicControlGUI* 		m_basicCtrlGui;
				SequencerGUI* 			m_sequencerGui;
				SupervisorGUI* 			m_supervisorGui;
				PlotGUI* 				m_plotGui;

				common::Robot* 			m_pRobot;
				QFile*					m_pSavefile;

				common::Project*		m_pProjet;
		};
	}  // namespace gui
}  // namespace osa_gui

#endif // osa_gui_MAIN_WINDOW_H
