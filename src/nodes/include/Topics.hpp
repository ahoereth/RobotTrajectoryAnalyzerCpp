//////
// nodes > Topics.hpp

#ifndef __TOPICS_INCLUDE__
#define __TOPICS_INCLUDE__


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


class Topics {
 private:
  ros::NodeHandle& node;
  AnnotationGateway& gateway;
  std::vector<int> joints;
  ros::Rate rate;
  bool loop;
  bool echo;

  bool isMovement(
    uima::ANIterator& iter,
    const uima::Feature& nameFtr,
    const uima::UnicodeStringRef& name,
    std::size_t position
  );

 public:
  static const std::vector<std::string> getPlots();
  static std::string getCommand(const std::string& plot, int size);

  Topics(
    ros::NodeHandle& node,
    AnnotationGateway& gateway,
    const std::pair< std::vector<int>, std::vector<std::string> >& joints,
    int rate = 1000,
    bool loop = true,
    bool echo = false
  );
  Topics(
    ros::NodeHandle& node,
    AnnotationGateway& gateway,
    const std::pair< std::vector<int>, std::vector<std::string> >& joints
  );
  void plot(const std::string& plot);
  void acc();
  void posvel();
  void posvelmov();
};

#endif  // ifndef __TOPICS_INCLUDE__
