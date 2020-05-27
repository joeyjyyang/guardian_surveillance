#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "guardian_surveillance/Object.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageProcessor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher object_pub_;
  guardian_surveillance::Object object_msg_;
 
public:
  ImageProcessor(const ros::NodeHandle& nh) : it_(nh), nh_(nh)
  {
    image_sub_ = it_.subscribe("/raspicam_node/image", 1,
      &ImageProcessor::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    object_pub_ = nh_.advertise<guardian_surveillance::Object>("/guardian_surveillance/object", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageProcessor()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /* Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0)); */

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());

    object_msg_.stamp = ros::Time::now();
    object_msg_.classification = "Human";
    object_msg_.confidence = 0.99;
    object_pub_.publish(object_msg_); 
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "image_processor_node");
  ros::NodeHandle nh;
  ImageProcessor ip(nh);
  ros::spin();
  return 0;
}
