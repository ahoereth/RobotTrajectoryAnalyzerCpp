//////
// rta > nodes > acceleration_topic.cpp

#include <string>
#include <vector>
#include <cstring>
#include "unicode/unistr.h"  // UnicodeString"
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "utils.hpp"
#include "AnnotationGateway.hpp"


static const std::string topic = "rtaAcceleration";


int main(int argc, char* argv[]) {
  // Initialize ros and the node.
  ros::init(argc, argv, topic);
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<std_msgs::Float64MultiArray>(topic, 1000);
  ros::Rate rate(100);

  // Handle -loop flag
  bool loop = (1 < argc && 0 == std::strcmp(argv[1], "-loop"));

  // Initalize and run annotators
  AnnotationGateway gateway = AnnotationGateway();
  gateway.run();
  AnnotationIterator accIter = gateway.getAnnotationIterator("Acceleration");

  // Initalize message with its layout.
  std_msgs::Float64MultiArray msg;
  std_msgs::MultiArrayLayout layout = std_msgs::MultiArrayLayout();
  std::vector<std_msgs::MultiArrayDimension> dimensions(1);
  dimensions[0].label = "accelerations";
  dimensions[0].size = accIter.getDoubleVector("value").size();
  dimensions[0].stride = 1;
  layout.dim = dimensions;
  msg.layout = layout;

  // Start publishing.
  while (node.ok() && (loop || accIter.isValid())) {
    if (loop && !accIter.isValid()) {
      accIter = gateway.getAnnotationIterator("Acceleration");
    }

    msg.data.clear();
    msg.data = accIter.getDoubleVector("value");
    pub.publish(msg);

    accIter.moveToNext();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
