//////
// rta > nodes > acceleration_topic.cpp

#include <string>
#include <vector>
#include <cstring>
#include <iomanip>
#include <iostream>
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
      std::string arg = argv[i];
      joints = utils::split(arg.substr(7), ',');
    }
  }

  // Initalize and run annotators
  AnnotationGateway gateway = AnnotationGateway();
  gateway.run();

  // Get required iterators.
  AnnotationIterator accIter = gateway.getAnnotationIterator("Acceleration");
  AnnotationIterator jsIter = gateway.getAnnotationIterator("JointState");
  std::vector<std::string> jointnames = jsIter.getStringVector("jointNames");
  std::vector<int> indices;

  // Handle just specific joint names if requested.
  if (joints.size() > 0) {  // Some joint names were passed as CL argument.
    // Get the indices of the requested joints - if any.
    for (int i = 0, size = joints.size(); i < size; i++) {
      int index = utils::indexOf(jointnames, joints[i]);
      if (index > -1) {
        indices.push_back(index);
      }
    }
  } else {  // Give the option to choose from a list of joint names.
    std::cout << "-----------------------------------------------" << std::endl;
    for (int i = 0, size = jointnames.size(); i < size; i++) {
      std::cout << std::left << std::setw(4) << utils::toString(i) + ") "
                             << std::setw(36) << jointnames[i];
      if (i%2 == 1) { std::cout << std::endl; }
    }
    std::cout << std::endl;

    std::string choice;
    std::cout << "Choose (all): ";
    std::getline(std::cin, choice);
    std::cout << "-----------------------------------------------" << std::endl;
    indices = utils::toInt(utils::split(choice, ','));
  }

  // Initalize message with its layout.
  std_msgs::Float64MultiArray msg;
  std_msgs::MultiArrayLayout layout = std_msgs::MultiArrayLayout();
  std::vector<std_msgs::MultiArrayDimension> dimensions(1);
  dimensions[0].label = "accelerations";  // Maybe list the joint names here?
  dimensions[0].stride = 1;
  dimensions[0].size = indices.size() == 0 ?
    accIter.getDoubleVector("values").size() : indices.size();
  layout.dim = dimensions;
  msg.layout = layout;

  // Start publishing.
  while (node.ok() && (loop || accIter.isValid())) {
    if (loop && !accIter.isValid()) {
      accIter = gateway.getAnnotationIterator("Acceleration");
    }

    msg.data.clear();
    if (indices.size() == 0) {  // All joints!
      msg.data = accIter.getDoubleVector("values");
    } else {  // Just specific joints.
      std::vector<double> values = accIter.getDoubleVector("values");
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
