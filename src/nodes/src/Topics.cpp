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
  std::vector<std::string> plots(4);
  plots[0] = "Acceleration";
  plots[1] = "Position + Velocity";
  plots[2] = "Position + Velocity + Movement";
  plots[3] = "Position Error";
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
  std::vector<std::string> plots = getPlots();
  std::vector<std::string> topics;

  if (plot == plots[0]) {
    topics.push_back("acc");
  } else if (plot == plots[1]) {
    topics.push_back("pos");
    topics.push_back("vel");
  } else if (plot == plots[2]) {
    topics.push_back("pos");
    topics.push_back("vel");
    topics.push_back("pmov");
    topics.push_back("nmov");
  } else if (plot == plots[3]) {
    topics.push_back("desired");
    topics.push_back("actual");
    topics.push_back("error");
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
    jointNames(joints.second), rate(rate), loop(loop), echo(echo)
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
  std::vector<std::string> plots = getPlots();

  if (plot == plots[0]) {
    acc();
  } else if (plot == plots[1]) {
    posvel();
  } else if (plot == plots[2]) {
    posvelmov();
  } else if (plot == plots[3]) {
    poserr();
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


/**
 * Fire off the position + velocitiy + movement direction topic.
 */
void Topics::posvelmov() {
  ros::Publisher posPub, velPub, pmovPub, nmovPub;
  posPub = node.advertise<std_msgs::Float64MultiArray>("pos", 1000);
  velPub = node.advertise<std_msgs::Float64MultiArray>("vel", 1000);
  pmovPub = node.advertise<std_msgs::Float64MultiArray>("pmov", 1000);
  nmovPub = node.advertise<std_msgs::Float64MultiArray>("nmov", 1000);

  std_msgs::Float64MultiArray posMsg;
  std_msgs::Float64MultiArray velMsg;
  std_msgs::Float64MultiArray pmovMsg;
  std_msgs::Float64MultiArray nmovMsg;

  uima::ANIterator jsIter = gateway.getANIterator("JointState");
  uima::Feature jsnF = gateway.getFeature("JointState", "jointNames");
  uima::Feature jtpF = gateway.getFeature("JointState", "jointTrajectoryPoint");
  uima::Feature posF = gateway.getFeature("JointTrajectoryPoint", "positions");
  uima::Feature velF = gateway.getFeature("JointTrajectoryPoint", "velocities");

  uima::ANIterator pmovIter = gateway.getANIterator("PositiveMovement");
  uima::ANIterator nmovIter = gateway.getANIterator("NegativeMovement");
  uima::Feature pmovnF = gateway.getFeature("PositiveMovement", "jointName");
  uima::Feature nmovnF = gateway.getFeature("NegativeMovement", "jointName");

  uima::AnnotationFS js, pmov, nmov;
  uima::FeatureStructure jtp;
  uima::StringArrayFS names;
  uima::DoubleArrayFS positions, velocities;

  std::vector<int>::const_iterator it;
  std::size_t i;

  while (node.ok() && (jsIter.isValid() || loop)) {
    if (!jsIter.isValid()) {
      jsIter = gateway.getANIterator("JointState");
    }

    js = jsIter.get();
    names = js.getStringArrayFSValue(jsnF);
    jtp = js.getFSValue(jtpF);
    positions = jtp.getDoubleArrayFSValue(posF);
    velocities = jtp.getDoubleArrayFSValue(velF);

    // Gene fresh message.
    posMsg.data.clear();
    velMsg.data.clear();
    pmovMsg.data.clear();
    nmovMsg.data.clear();
    posMsg.data.resize(joints.size());
    velMsg.data.resize(joints.size());
    pmovMsg.data.resize(joints.size());
    nmovMsg.data.resize(joints.size());
    for (it = joints.begin(), i = 0; it != joints.end(); it++, i++) {
      posMsg.data[i] = positions.get(*it);
      velMsg.data[i] = velocities.get(*it);

      if (isMovement(pmovIter, pmovnF, names.get(*it), js.getBeginPosition())) {
        pmovMsg.data[i] = positions.get(*it);
      }

      if (isMovement(nmovIter, nmovnF, names.get(*it), js.getBeginPosition())) {
        nmovMsg.data[i] = positions.get(*it);
      }
    }

    // Print current message data.
    if (echo) {
      std::puts(utils::join(posMsg.data, " | ").c_str());
      std::puts(utils::join(velMsg.data, " | ").c_str());
      std::puts(utils::join(pmovMsg.data, " | ").c_str());
      std::puts(utils::join(nmovMsg.data, " | ").c_str());
      std::puts("\n");
    }

    posPub.publish(posMsg);
    velPub.publish(velMsg);
    pmovPub.publish(pmovMsg);
    nmovPub.publish(nmovMsg);

    jsIter.moveToNext();
    ros::spinOnce();
    rate.sleep();
  }
}


/**
 * Fire off desired, actual and error position topics.
 */
void Topics::poserr() {
  uima::ANIterator ciIter = gateway.getANIterator("ControllerInput");
  uima::Feature namesFtr = gateway.getFeature("ControllerInput", "jointNames");
  uima::Feature desiredFtr = gateway.getFeature("ControllerInput", "desired");
  uima::Feature actualFtr  = gateway.getFeature("ControllerInput", "actual");
  uima::Feature errorFtr   = gateway.getFeature("ControllerInput", "error");
  uima::Feature pFtr = gateway.getFeature("JointTrajectoryPoint", "positions");

  ros::Publisher desired, actual, error;
  desired = node.advertise<std_msgs::Float64MultiArray>("desired", 1000);
  actual  = node.advertise<std_msgs::Float64MultiArray>("actual", 1000);
  error   = node.advertise<std_msgs::Float64MultiArray>("error", 1000);

  std_msgs::Float64MultiArray desiredMsg, actualMsg, errorMsg;

  uima::AnnotationFS ci;
  uima::StringArrayFS names;
  uima::DoubleArrayFS desiredVal, actualVal, errorVal;

  std::vector<std::string>::const_iterator namesIt;
  std::size_t i = 0;

  while (node.ok() && (ciIter.isValid() || loop)) {
    if (!ciIter.isValid()) {
      ciIter = gateway.getANIterator("ControllerInput");
    }

    ci = ciIter.get();
    ciIter.moveToNext();
    names = ci.getStringArrayFSValue(namesFtr);
    desiredVal = ci.getFSValue(desiredFtr).getDoubleArrayFSValue(pFtr);
    actualVal  = ci.getFSValue(actualFtr).getDoubleArrayFSValue(pFtr);
    errorVal   = ci.getFSValue(errorFtr).getDoubleArrayFSValue(pFtr);

    // Generate fresh message.
    desiredMsg.data.clear();
    actualMsg.data.clear();
    errorMsg.data.clear();
    desiredMsg.data.resize(joints.size());
    actualMsg.data.resize(joints.size());
    errorMsg.data.resize(joints.size());

    bool gotdata = false;
    for (
      namesIt = jointNames.begin(), i = 0;
      jointNames.end() != namesIt; namesIt++, i++
    ) {
      for (std::size_t j = 0; j < names.size(); j++) {
        if (utils::toString(names.get(j).getBuffer()) == *namesIt) {
          desiredMsg.data[i] = desiredVal.get(j);
          actualMsg.data[i]  = actualVal.get(j);
          errorMsg.data[i]   = errorVal.get(j);
          gotdata = true;
        }
      }
    }

    if (!gotdata) {
      continue;
    }

    // Print current message data.
    if (echo) {
      std::puts(utils::join(desiredMsg.data, " | ").c_str());
      std::puts(utils::join(actualMsg.data, " | ").c_str());
      std::puts(utils::join(errorMsg.data, " | ").c_str());
      std::puts("");
    }

    desired.publish(desiredMsg);
    actual.publish(actualMsg);
    error.publish(errorMsg);

    ros::spinOnce();
    rate.sleep();
  }
}


/**
 * Check if the given joint name has a movement annotation associated with it.
 *
 * @param  iter     An Annotation Iterator over `Movement`, `PositiveMovement`
 *                  or `NegativeMovement` annotations.
 * @param  nameFtr  The jointName feature of the specific annotation.
 * @param  name     The joint we are looking for.
 * @param  position The joint `BeginPosition` or `EndPosition` annotation.
 * @return
 */
bool Topics::isMovement(
  uima::ANIterator& iter,
  const uima::Feature& nameFtr,
  const uima::UnicodeStringRef& name,
  std::size_t position
) {
  uima::AnnotationFS mov;
  iter.moveToFirst();

  // Iterate over all movements in order to find the one which is relevant.
  do {
    if (!iter.isValid()) {
      break;
    }

    mov = iter.get();
    iter.moveToNext();
  } while (
    mov.getStringValue(nameFtr) != name ||  // Wrong joint.
    mov.getBeginPosition() > position   ||  // Too early.
    mov.getEndPosition()   < position       // Too late.
  );

  // If the iterator is still valid we found a relevant annotation.
  if (iter.isValid()) {
    return true;
  } else {
    return false;
  }
}
