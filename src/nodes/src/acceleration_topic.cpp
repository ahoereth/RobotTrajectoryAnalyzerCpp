//////
// rta > nodes > acceleration_topic.cpp

#include <string>
#include <vector>
#include "unicode/unistr.h"  // UnicodeString"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "utils.hpp"
#include "AnnotationGateway.hpp"


static const std::string topic = "rtaAcceleration";


int main(int argc, char **argv) {
  ros::init(argc, argv, topic);

  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<std_msgs::Float64MultiArray>(topic, 1000);

  AnnotationGateway gateway = AnnotationGateway();
  gateway.run();

  ros::Rate rate(10);
  AnnotationIterator accIter = gateway.getAnnotationIterator("Acceleration");

  while (node.ok() && accIter.isValid()) {
    std_msgs::Float64MultiArray arr;
    arr.data = accIter.getDoubleVector("value");
    pub.publish(arr);

    accIter.moveToNext();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
