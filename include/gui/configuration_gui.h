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
 * @file configuration_gui.h
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version OSA 0.1.0
 * @brief Header file for Qt based gui class ConfigurationGUI.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

#ifndef OSA_GUI_GUI_CONFIGURATION_GUI_H
#define OSA_GUI_GUI_CONFIGURATION_GUI_H

#include <QMdiSubWindow>
#include "ui_ConfigurationGUI.h"

namespace osa_gui
{
namespace gui
{

/**
 * @brief This defines the Qt GUI for the configuration window.
 */
class ConfigurationGUI : public QMdiSubWindow
{
Q_OBJECT

public:
	/**
	 * @brief Constructor.
	 */
	ConfigurationGUI(QWidget *parent = 0);

	/**
	 * @brief Destructor.
	 */
	~ConfigurationGUI();

public Q_SLOTS:
	/*! Auto-connections (connectSlotsByName()) */
	void on_treev_robot_itemClicked(QTreeWidgetItem *item, int column);
	void on_cb_type_currentIndexChanged(int index);
	void on_pb_addHardware_clicked();
	void on_pb_wizard_clicked();

private:
	void addTreeRoot(QString name, QString description);
	void addTreeChild(QTreeWidgetItem *parent, QString name, QString description);

private:
	Ui::ConfigurationGUIClass ui_;
};

} // namespace gui
} // namespace osa_gui

#endif // OSA_GUI_GUI_CONFIGURATION_GUI_H
