#include <ros/ros.h>

class IntruderAlerter
{
  ros::NodeHandle nh_;
  ros::Subscriber object_pub_;
 
public:
  IntruderAlerter()
    : it_(nh_)
  {
    object_sub_ = nh_.subscribe("/guardian_surveillance/object", 1, &IntruderAlerter::objectCb, this);
  }

  ~IntruderAlerter()
  {
  }

  void objectCb(const guardian_surveillance::Object::ConstPtr& object_msg)
  {
    ROS_INFO("test"); 
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "intruder_alerter_node");
  IntruderAlerter ia;
  ros::spin();
  return 0;
}
