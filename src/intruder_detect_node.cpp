#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <stdio.h>

std::string face_cascade_name = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_alt.xml";
cv::CascadeClassifier face_cascade;

static const std::string OPENCV_WINDOW = "Guardian Surveillance";

class ImageProcessor
{
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::ServiceClient email_alert_client_;
  std_srvs::Empty email_alert_srv_;
  std::vector<cv::Rect> faces_;
  int face_count_;
 
public:
  ImageProcessor(const ros::NodeHandle& nh) : nh_(nh), face_count_(0)
  {
    image_sub_ = nh_.subscribe("/raspicam_node/image/compressed", 1, &ImageProcessor::imageCb, this);
    email_alert_client_ = nh_.serviceClient<std_srvs::Empty>("email_alert");
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageProcessor()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    cv::Mat cv_image = cv::imdecode(cv::Mat(msg->data), 1); 

    cv::Mat frame_gray;
    cv::cvtColor(cv_image, frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);

    face_cascade.detectMultiScale(frame_gray, faces_);
    
    if (!faces_.empty() && faces_.size() != face_count_)
    {
      face_count_ = faces_.size();
      ROS_INFO("Warning! Intruder(s) Detected!");
      if (email_alert_client_.call(email_alert_srv_))
      {
        ROS_INFO("IMAGE SAVED");
      }
      else
      {
        ROS_ERROR("FAILED TO CALL SERVICE email_alert");
      }
    }
    
    for (size_t i = 0; i < faces_.size(); i++)
    {
      cv::Point center(faces_[i].x + faces_[i].width/2, faces_[i].y + faces_[i].height/2);
      cv::ellipse(cv_image, center, cv::Size(faces_[i].width/2, faces_[i].height/2), 0, 0, 360, cv::Scalar(255, 0, 255), 4);
    }
	
    cv::imshow(OPENCV_WINDOW, cv_image);
    cv::waitKey(3);
  }
};

int main(int argc, char* argv[])
{

  if (!face_cascade.load(face_cascade_name))
  {
    std::cout << "Error: Unable to locate Haar Cascade xml files!" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "intruder_detect_node");
  ros::NodeHandle nh;
  ImageProcessor ip(nh);
  ros::spin();
  return 0;
}
