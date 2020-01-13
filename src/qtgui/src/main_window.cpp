/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QTime>
#include "../include/qtgui/main_window.hpp"
#include <QtCore>
#include <QFile>
#include <QDebug>
#include <QTableView>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtgui {

  using namespace Qt;

extern std::string Vicon;
extern std::string Ur5;
extern std::string Capture;
extern std::vector<double> joint_angles;
extern std::vector<double> joint_t_angles;
extern uint joint_flag;
std::string flag = " ";
QList<double> List;
QList<double> TList;
QTimer *timer = new QTimer();
QStandardItemModel *model = new QStandardItemModel();
void sleepT(int sec)
{

  QTime rt = QTime::currentTime().addSecs(sec);
  while(QTime::currentTime() < rt);
}
  /*****************************************************************************
  ** Implementation [MainWindow]
  *****************************************************************************/
  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
  {
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    setWindowIcon(QIcon(":/images/icon.png"));
    system("gnome-terminal -x bash -c 'roscore' &");
    sleepT(8);

    while(!qnode.init());

    ui.tableView->setModel(model);
    model->setHorizontalHeaderItem(0, new QStandardItem(tr("State_1")));
    model->setHorizontalHeaderItem(1, new QStandardItem(tr("State_2")));
    model->setHorizontalHeaderItem(2, new QStandardItem(tr("State_3")));
    model->setHorizontalHeaderItem(3, new QStandardItem(tr("State_4")));
    model->setHorizontalHeaderItem(4, new QStandardItem(tr("State_5")));
    model->setHorizontalHeaderItem(5, new QStandardItem(tr("State_6")));
    model->setVerticalHeaderItem(0,new QStandardItem(tr("target (rad)")));
    model->setVerticalHeaderItem(1,new QStandardItem(tr("current (rad)")));
    ui.tableView->setColumnWidth(0, 70);
    ui.tableView->setColumnWidth(1, 70);
    ui.tableView->setColumnWidth(2, 70);
    ui.tableView->setColumnWidth(3, 70);
    ui.tableView->setColumnWidth(4, 70);
    ui.tableView->setColumnWidth(5, 70);
    ui.tableView->setRowHeight(0, 30);
    ui.tableView->setRowHeight(1, 30);


    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpDate()));
    timer->start(20);
  }

  void MainWindow::timerUpDate()

  {

    for (unsigned int i = 0; i < 6; i++) {
      QStandardItem *item = new QStandardItem(QString::number(joint_angles[i],'f',2));
      model->setItem(1,i,item);

      QStandardItem *titem = new QStandardItem(QString::number(joint_t_angles[i],'f',2));
      model->setItem(0,i,titem);
    }
    if(joint_flag > 0)
    {
      List.append(joint_angles[0]);
      TList.append(joint_t_angles[0]);
      update();
    }



    QString text = QString::fromStdString(Vicon);
    ui.Vicon->setText(text);

    text = QString::fromStdString(Capture);
    ui.Track->setText(text);

    text = QString::fromStdString(Ur5);
    ui.UR5->setText(text);
  }

  MainWindow::~MainWindow() {}


  void MainWindow::paintEvent(QPaintEvent *event)
  {
    QPainter painter(this);
    QPen pen;
    QFont font;
    font.setPointSize(4);
    pen.setColor(Qt::black);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.setFont(font);
    painter.setWindow(0,0,400,300);
    painter.drawLine(30,215,390,215);
    painter.drawLine(30,140,30,290);

    painter.drawLine(320,130,340,130);
    painter.drawText(345, 131, "current_state");


    painter.drawText(25, 136, tr("(rad)"));
    painter.drawText(394, 224, tr("(s)"));
    for(uint j=0;j<9;j++)
    {
      painter.drawLine(27, 140+j*18.75, 30, 140+j*18.75);
      painter.drawText(16, 142+j*18.75, QString::number((2-0.5*j)*3.14));
      painter.drawLine(30+(j+1)*20, 215, 30+(j+1)*20, 218);
      painter.drawText(27+(j+1)*20, 224, QString::number((j+1)*5));
      painter.drawLine(30+(j+10)*20, 215, 30+(j+10)*20, 218);
      painter.drawText(27+(j+10)*20, 224, QString::number((j+10)*5));
    }
    pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.drawLine(320,120,340,120);
    painter.drawText(345, 122, "target_state");
    for(int i = 0; i < List.count(); i++)
        {
            if(i == 0)
            {
              painter.drawPoint(QPointF(30, 215-(List[i]/6.28)*75));
              painter.drawPoint(QPointF(30, 215-(TList[i]/6.28)*75));
            }
            else
            {
              pen.setColor(Qt::red);
              painter.setPen(pen);
              painter.drawLine(QPointF(30+(i-1)*0.08, 215-(TList[i-1]/6.28)*75), QPointF(30+i*0.08, 215-(TList[i]/6.28)*75));
              pen.setColor(Qt::black);
              painter.setPen(pen);
              painter.drawLine(QPointF(30+(i-1)*0.08, 215-(List[i-1]/6.28)*75), QPointF(30+i*0.08, 215-(List[i]/6.28)*75));
            }
        }
  }



  void MainWindow::closeEvent(QCloseEvent *event)
  {
    QMainWindow::closeEvent(event);
  }

}  // namespace qtgui




void qtgui::MainWindow::on_Start_clicked()
{
    system("gnome-terminal -x bash -c 'roslaunch vicon_bridge vicon.launch' ");
    sleepT(1);
    system("gnome-terminal -x bash -c 'roslaunch ur_modern_driver ur5_bringup.launch' ");
    sleepT(5);
    system("gnome-terminal -x bash -c 'roslaunch vicon_bridge calibration_result.launch' ");
}

void qtgui::MainWindow::on_Stop_clicked()
{
    if(timer->isActive())
    {
      timer->stop();
      ui.Stop->setText("Start");
    }
    else
    {
      timer->start();
      ui.Stop->setText("Stop");
    }

}

void qtgui::MainWindow::on_Teaching_clicked()
{
   joint_flag = 1;
   system("gnome-terminal -x bash -c 'roslaunch trac_ik_examples ur_5.launch' ");
   sleepT(4);
   List.clear();
   TList.clear();
}

void qtgui::MainWindow::on_Capture_clicked()
{
   system("gnome-terminal -x bash -c 'rosrun vicon_bridge pen_convert' ");
}

void qtgui::MainWindow::on_Offline_clicked()
{
    joint_flag = 2;
    List.clear();
    TList.clear();
}
