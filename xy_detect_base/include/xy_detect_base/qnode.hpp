/**
 * @file /include/xy_detect_base/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef xy_detect_base_QNODE_HPP_
#define xy_detect_base_QNODE_HPP_

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
#include <QStringListModel>
#include <QString>
#include <sensor_msgs/Image.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <mobile_base_msgs/mani_vision.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xy_detect_base
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  cv::Mat* imgRaw = NULL;                                         // 원본 이미지를 가리키는 포인터
  bool isreceived = false;                                        // 수신 여부를 나타내는 플래그
  image_transport::Subscriber subImage;                           // 서브스크라이버
  void callbackImage(const sensor_msgs::ImageConstPtr& msg_img);  // 이미지 콜백 함수 선언

  ros::Publisher mani_vision_pub;

Q_SIGNALS:
  void rosShutdown();
  void sigRcvImg();

private:
  int init_argc;
  char** init_argv;
};

}  // namespace xy_detect_base

#endif /* xy_detect_base_QNODE_HPP_ */
