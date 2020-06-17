#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <stdio.h>

#include "guardian_surveillance/Object.h"

std::string face_cascade_name = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_alt.xml";
std::string eyes_cascade_name = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eyes_cascade;

static const std::string OPENCV_WINDOW = "Guardian Surveillance";

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

    cv::Mat frame_gray;
    cv::cvtColor(cv_ptr->image, frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);

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

  if (!face_cascade.load(face_cascade_name) || !eyes_cascade.load(eyes_cascade_name))
  {
    std::cout << "Error: Unable to locate Haar Cascade xml files!" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "image_processor_node");
  ros::NodeHandle nh;
  ImageProcessor ip(nh);
  ros::spin();
  return 0;
}
