/**
 * @file /include/qtgui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtgui_QNODE_HPP_
#define qtgui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <std_msgs/String.h>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtgui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
 // std::string ROSt;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
  //void conncallback(const std::string& chat);
  std::string ttf(std::string pms);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  ros::Subscriber ur5_subscriber;
  ros::Subscriber joint_subscriber;
  ros::Subscriber Vicon_subscriber;
  ros::Subscriber pen_subscriber;
  ros::Subscriber joint_t_subscriber;
    QStringListModel logging_model;
};

}  // namespace qtgui

#endif /* qtgui_QNODE_HPP_ */
