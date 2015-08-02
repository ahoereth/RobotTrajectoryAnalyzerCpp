//////
// mongourdf > ModelState.hpp

#ifndef __MODELSTATE_INCLUDE__
#define __MODELSTATE_INCLUDE__


#include <string>
#include <vector>
#include "JointState.hpp"


/**
 * @see http://wiki.ros.org/urdf/XML/model_state
 */
class ModelState {
 public:
  std::string name;
  long time;
  std::vector<JointState> jointStates;

  ModelState(const std::string& name, double time) : name(name), time(time) {}
  ~ModelState(void) {}
  void addJointState(const JointState& jointState) {
    jointStates.push_back(jointState);
  }
};


#endif  // ifndef __MODELSTATE_INCLUDE__
