#include <ros/ros.h>

#include "guardian_surveillance/Object.h"

class IntruderAlerter
{
  ros::NodeHandle nh_;
  ros::Subscriber object_sub_;
 
public:
  IntruderAlerter(const ros::NodeHandle& nh) : nh_(nh)
  {
    object_sub_ = nh_.subscribe("/guardian_surveillance/object", 1, &IntruderAlerter::objectCb, this);
  }

  ~IntruderAlerter()
  {
  }

  void objectCb(const guardian_surveillance::Object::ConstPtr& object_msg)
  {
    if (object_msg->classification == "Human" && object_msg->confidence >= 0.9)
    {
      /* bluetooth ping phone */
      ROS_INFO("WARNING: INTRUDER DETECTED!"); 
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "intruder_alerter_node");
  ros::NodeHandle nh;
  IntruderAlerter ia(nh);
  ros::spin();
  return 0;
}
