/**
 * @file /include/qtgui/main_window.hpp
 *
 * @brief Qt based gui for qtgui.
 *
 * @date November 2010
 **/
#ifndef qtgui_MAIN_WINDOW_H
#define qtgui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qtgui {

  /*****************************************************************************
  ** Interface [MainWindow]
  *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow {
  Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    void closeEvent(QCloseEvent *event); // Overloaded function
    void paintEvent(QPaintEvent *event);

  public Q_SLOTS:
    void on_Start_clicked();
    void on_Stop_clicked();
    void timerUpDate();
    void on_Teaching_clicked();

    void on_Capture_clicked();
    void on_Offline_clicked();


  private:
    Ui::ROS_GUI ui;
    QNode qnode;
  };

}  // namespace qtgui

#endif // qtgui_MAIN_WINDOW_H
