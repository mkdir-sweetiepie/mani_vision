#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "librealsense2/rsutil.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <mobile_base_msgs/mani_vision.h>

bool isReceived = false;

class Depth
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  ros::Subscriber in_center_sub;
  ros::Publisher distance_pub;
  mobile_base_msgs::mani_vision mvis;

public:
  Depth() : it_(nh_)
  {
    std::string image_topic = nh_.resolveName("camera/aligned_depth_to_color/image_raw");
    sub_ = it_.subscribeCamera(image_topic, 1024, &Depth::imageCb, this);
    in_center_sub = nh_.subscribe<mobile_base_msgs::mani_vision>("xy_detect", 1, &Depth::center_check_Callback, this);
    distance_pub = nh_.advertise<mobile_base_msgs::mani_vision>("xy_detect", 100);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    
    if (isReceived)
    {
      cv::Mat image;
      cv_bridge::CvImagePtr input_bridge;
      try
      {
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        image = input_bridge->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("[draw_frames] Failed to convert image");
        return;
      }

      // 이미지의 중심 좌표 계산
      int center_x = image.cols / 2;
      int center_y = image.rows / 2;

      float depth_in_mm = image.at<short int>(cv::Point(center_x, center_y));

      mvis.center_depth = depth_in_mm;
      // 나중에 수정할 부분
      mvis.high_depth = depth_in_mm;
      mvis.low_depth = depth_in_mm;
      distance_pub.publish(mvis);
      
      std::cout << "distance : " << depth_in_mm << std::endl;

      isReceived = false;
    }
  }

  void center_check_Callback(const mobile_base_msgs::mani_visionConstPtr& msg)
  {
    if (msg->in_center == true)
    {
      isReceived = true;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Depth");
  Depth a;
  ros::NodeHandle n;
  ros::spin();
}
