#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <xmlrpcpp/XmlRpcValue.h> // catkin component

namespace realsense2_camera
{

class YAMLParser
{
public:
  YAMLParser();
  virtual ~YAMLParser();
  void parseYAML(ros::NodeHandle& nh);
};

}  // namespace realsense2_camera