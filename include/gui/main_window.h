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
 * @file main_window.h
 * @author Cyril Jourdan
 * @date Mar 15, 2017
 * @version OSA 0.1.0
 * @brief Header file for Qt based gui class MainWindow.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Feb 20, 2017
 */

#ifndef OSA_GUI_GUI_MAIN_WINDOW_H
#define OSA_GUI_GUI_MAIN_WINDOW_H

#include <QFile>
#include <include/gui/configuration_gui.h>
#include <include/gui/basic_control_gui.h>
#include <include/gui/sequencer_gui.h>
#include <include/gui/plot_gui.h>
#include <include/gui/supervisor_gui.h>
#include <include/common/robot.h>
#include <include/common/project.h>
#include <QMainWindow>
#include "ui_MainWindow.h"

namespace osa_gui
{
namespace gui
{
//typedef QList<int> StatusArray;

/**
 * @brief This defines the Qt GUI for the main window.
 */
class MainWindow : public QMainWindow
{
Q_OBJECT

public:

	/**
	 * @brief Constructor.
	 */
	MainWindow(int argc, char** argv, QWidget *parent = 0);

	/**
	 * @brief Destructor.
	 */
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
	Ui::MainWindowDesign ui_;
	ConfigurationGUI* ptr_configuration_gui_;
	BasicControlGUI* ptr_basic_control_gui_;
	SequencerGUI* ptr_sequencer_gui_;
	SupervisorGUI* ptr_supervisor_gui_;
	PlotGUI* ptr_plot_gui_;
	common::Robot* ptr_robot_;
	QFile* ptr_saved_file_;
	common::Project* ptr_projet_;
};

}  // namespace gui
}  // namespace osa_gui

#endif // OSA_GUI_GUI_MAIN_WINDOW_H
