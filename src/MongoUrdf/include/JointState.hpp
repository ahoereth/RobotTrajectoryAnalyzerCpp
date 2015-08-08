//////
// mongourdf > JointState.hpp

#ifndef __JOINTSTATE_INCLUDE__
#define __JOINTSTATE_INCLUDE__


#include <string>


/**
 * @see http://wiki.ros.org/urdf/XML/model_state
 */
class JointState {
 public:
  std::string name;
  double position;
  double velocity;
  double effort;
  double acceleration;

  explicit JointState(const std::string& name) : name(name) {}
  JointState(
    const std::string& name, double position, double velocity, double effort)
    : name(name), position(position), velocity(velocity), effort(effort) {}
  JointState(
    const std::string& name, double position,
    double velocity, double effort, double acceleration) :
    name(name), position(position),
    velocity(velocity), effort(effort), acceleration(acceleration) {}
  ~JointState(void) {}
};


#endif  // ifndef __JOINTSTATE_INCLUDE__
