//////
// nodes > Topics.cpp

#include <vector>
#include <cstring>  // size_t
#include <cstdio>  // printf, puts
#include <utility>  // pair, make_pair
#include "unicode/unistr.h"  // UnicodeString
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "uimautils.hpp"
#include "AnnotationGateway.hpp"
#include "Topics.hpp"


/**
 * Get a vector of all available plots.
 *
 * @return Vector of plot names as strings.
 */
const std::vector<std::string> Topics::getPlots() {
  std::vector<std::string> plots(2);
  plots[0] = "acc";
  plots[1] = "posvel";
  return plots;
}


/**
 * Get the rqt_plot command required for visualizing the specified topic.
 *
 * @param  plot Plot to visualize.
 * @param  size Amount of joints visualized.
 * @return Terminal command to be executed.
 */
std::string Topics::getCommand(const std::string& plot, int size) {
  std::vector<std::string> topics;

  if ("acc" == plot) {
    topics.push_back("acc");
  } else if ("posvel" == plot) {
    topics.push_back("pos");
    topics.push_back("vel");
  }

  std::string cmd = "rqt_plot ";
  for (std::size_t i = 0; i < topics.size(); i++) {
    for (int idx = 0; idx < size; idx++) {
      cmd += "/" + topics[i] + "/data[" + utils::toString(idx) + "] ";
    }
  }

  return cmd;
}


/**
 * Constructor which initializes the required class variables and prints the
 * topic headline if desired.
 */
Topics::Topics(
  ros::NodeHandle& node,
  AnnotationGateway& gateway,
  const std::pair< std::vector<int>, std::vector<std::string> >& joints,
  int rate,
  bool loop,
  bool echo
) : node(node), gateway(gateway), joints(joints.first),
    rate(rate), loop(loop), echo(echo)
{
  if (echo) {
    std::puts(utils::join(joints.second, " | ").c_str());
  }
}


/**
 * Fire off the topic specified by the given parameter.
 *
 * @param plot
 */
void Topics::plot(const std::string& plot) {
  if ("acc" == plot) {
    acc();
  } else if ("posvel" == plot) {
    posvel();
  }
}


/**
 * Fire off the acceleration topic.
 */
void Topics::acc() {
  uima::ANIterator accIter = gateway.getANIterator("Acceleration");
  uima::Feature valuesFtr = gateway.getFeature("Acceleration", "values");
  ros::Publisher acc = node.advertise<std_msgs::Float64MultiArray>("acc", 1000);
  std_msgs::Float64MultiArray msg;

  uima::DoubleArrayFS values;

  std::vector<int>::const_iterator it;
  std::size_t i;

  while (node.ok() && (accIter.isValid() || loop)) {
    if (!accIter.isValid()) {
      accIter = gateway.getANIterator("Acceleration");
    }

    // Generate fresh message.
    msg.data.clear();
    msg.data.resize(joints.size());
    values = accIter.get().getDoubleArrayFSValue(valuesFtr);
    for (it = joints.begin(), i = 0; it != joints.end(); it++) {
      msg.data[i++] = values.get(*it);
    }

    // Print current message data.
    if (echo) {
      std::puts(utils::join(msg.data, " | ").c_str());
    }

    acc.publish(msg);
    accIter.moveToNext();
    ros::spinOnce();
    rate.sleep();
  }
}


/**
 * Fire off the position and velocitiy topic.
 */
void Topics::posvel() {
  ros::Publisher posPub, velPub;
  posPub = node.advertise<std_msgs::Float64MultiArray>("pos", 1000);
  velPub = node.advertise<std_msgs::Float64MultiArray>("vel", 1000);

  std_msgs::Float64MultiArray posMsg;
  std_msgs::Float64MultiArray velMsg;

  uima::ANIterator jsIter = gateway.getANIterator("JointState");
  uima::Feature jtpF = gateway.getFeature("JointState", "jointTrajectoryPoint");
  uima::Feature posF = gateway.getFeature("JointTrajectoryPoint", "positions");
  uima::Feature velF = gateway.getFeature("JointTrajectoryPoint", "velocities");
  uima::FeatureStructure jtp;
  uima::DoubleArrayFS positions, velocities;

  std::vector<int>::const_iterator it;
  std::size_t i;

  while (node.ok() && (jsIter.isValid() || loop)) {
    if (!jsIter.isValid()) {
      jsIter = gateway.getANIterator("JointState");
    }

    jtp = jsIter.get().getFSValue(jtpF);
    positions = jtp.getDoubleArrayFSValue(posF);
    velocities = jtp.getDoubleArrayFSValue(velF);

    // Gene fresh message.
    posMsg.data.clear();
    velMsg.data.clear();
    posMsg.data.resize(joints.size());
    velMsg.data.resize(joints.size());
    for (it = joints.begin(), i = 0; it != joints.end(); it++, i++) {
      posMsg.data[i] = positions.get(*it);
      velMsg.data[i] = velocities.get(*it);
    }

    // Print current message data.
    if (echo) {
      std::puts(utils::join(posMsg.data, " | ").c_str());
      std::puts(utils::join(velMsg.data, " | ").c_str());
      std::puts("\n");
    }

    posPub.publish(posMsg);
    velPub.publish(velMsg);

    jsIter.moveToNext();
    ros::spinOnce();
    rate.sleep();
  }
}
