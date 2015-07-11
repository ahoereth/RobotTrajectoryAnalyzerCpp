//////
// rta > nodes > acceleration_topic.cpp

#include <string>
#include <vector>
#include <cstring>
#include <sstream>
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

  // Handle command line arguments
  bool loop = false;
  std::vector<std::string> joints;
  for (int i = 1; i < argc; i++) {
    // -loop flag
    if (0 == std::strcmp(argv[i], "-loop")) {
      loop = true;
    }

    // joints list
    if (0 == std::strncmp(argv[i], "joints=", 7)) {
      std::string joint, arg = argv[i];
      std::istringstream ss(arg.substr(7));
      while (std::getline(ss, joint, ',')) {
        joints.push_back(joint);
      }
    }
  }

  // Initalize and run annotators
  AnnotationGateway gateway = AnnotationGateway();
  gateway.run();

  // Get required iterators.
  AnnotationIterator accIter = gateway.getAnnotationIterator("Acceleration");
  AnnotationIterator jsIter = gateway.getAnnotationIterator("JointState");
  std::vector<std::string> jointnames = jsIter.getStringVector("name");

  // Get the indices of the requested joints - if any.
  std::vector<int> indices;
  for (int i = 0, size = joints.size(); i < size; i++) {
    int index = utils::indexOf(jointnames, joints[i]);
    if (index > -1) {
      indices.push_back(index);
    }
  }

  // Initalize message with its layout.
  std_msgs::Float64MultiArray msg;
  std_msgs::MultiArrayLayout layout = std_msgs::MultiArrayLayout();
  std::vector<std_msgs::MultiArrayDimension> dimensions(1);
  dimensions[0].label = "accelerations";  // Maybe list the joint names here?
  dimensions[0].stride = 1;
  dimensions[0].size = indices.size() == 0 ?
    accIter.getDoubleVector("value").size() : indices.size();
  layout.dim = dimensions;
  msg.layout = layout;

  // Start publishing.
  while (node.ok() && (loop || accIter.isValid())) {
    if (loop && !accIter.isValid()) {
      accIter = gateway.getAnnotationIterator("Acceleration");
    }

    msg.data.clear();
    if (indices.size() == 0) {  // All joints!
      msg.data = accIter.getDoubleVector("value");
    } else {  // Just specific joints.
      std::vector<double> values = accIter.getDoubleVector("value");
      for (int i = 0, size = indices.size(); i < size; i++) {
        msg.data.push_back(values[indices[i]]);
      }
    }

    pub.publish(msg);

    accIter.moveToNext();
    jsIter.moveToNext();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
