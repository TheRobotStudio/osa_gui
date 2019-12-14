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
 * @file plot_gui.h
 * @author Cyril Jourdan
 * @date Mar 17, 2017
 * @version OSA 0.1.0
 * @brief Implementation file for class PlotGUI.
 * 		  Adapted from QCustomPlot interaction example.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 17, 2017
 */

#ifndef OSA_GUI_GUI_PLOT_GUI_H
#define OSA_GUI_GUI_PLOT_GUI_H

//#include <QMainWindow>
#include <QMdiSubWindow>
#include <QInputDialog>
#include "qcustomplot/qcustomplot.h"
#include "../rosnode/plot_node.h"
#include "ui_PlotGUI.h"

namespace osa_gui
{
namespace gui
{

/**
 * @brief This defines the Qt GUI for the plot window.
 */
class PlotGUI : public QMdiSubWindow //QMainWindow
{
Q_OBJECT

public:
	//explicit PlotGUI(QWidget *parent = 0);

	/**
	 * @brief Constructor.
	 */
	PlotGUI(QWidget *parent = 0);

	/**
	 * @brief Destructor.
	 */
	~PlotGUI();

private Q_SLOTS: //slots:
	void titleDoubleClick(QMouseEvent *event);
	void axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
	void legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
	void selectionChanged();
	void mousePress();
	void mouseWheel();
	//void addRandomGraph();
	void removeSelectedGraph();
	void removeAllGraphs();
	void contextMenuRequest(QPoint pos);
	void moveLegend();
	void graphClicked(QCPAbstractPlottable *plottable, int dataIndex);

	/*! Auto-connections (connectSlotsByName()) */
	//void on_ch_sel_0_stateChanged(int state);
	void on_pb_record_0_pressed();
	void on_pb_color_0_released();

	//slots of callbacks
	void setCustomPlotData();

private:
	Ui::PlotGUIClass ui_;
	rosnode::PlotNode plot_node_;
	int time_;
	QVector<double> x_; //x(5000), y(5000);
	QVector<double> y_;
	QVector<QColor> graph_color_;
};

} // namespace gui
} // namespace osa_gui

#endif // OSA_GUI_GUI_PLOT_GUI_H
