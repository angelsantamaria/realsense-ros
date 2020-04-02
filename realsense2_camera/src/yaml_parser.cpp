#include <yaml_parser.h>

using namespace realsense2_camera;

YAMLParser::YAMLParser()
{}

YAMLParser::~YAMLParser()
{}

void YAMLParser::parseYAML(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue params;

  // nh->getParam("camera", params);

  // if(params.getType() == XmlRpc::XmlRpcValue::Type::TypeArray && params.size() > 0){

  //   // boxes[0] is a 'TypeArray' aka vector
  //   if(boxes[0].getType() == XmlRpc::XmlRpcValue::Type::TypeArray && boxes[0].size() > 0){
  //     // boxes[0][0] is a 'TypeStruct' aka map
  //     if(boxes[0][0].getType() == XmlRpc::XmlRpcValue::Type::TypeStruct && boxes[0][0].hasMember("x")){
  //       x = double(boxes[0][0]["x"]);
  //       for(XmlRpc::XmlRpcValue::iterator it = boxes[0][0].begin(); it != boxes[0][0].end(); ++it){
  //         point[i++] = double(*it);
  //       }
  //     }
  //   }
  // }
}