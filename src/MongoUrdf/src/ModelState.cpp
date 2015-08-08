//////
// mongourdf > ModelState.cpp

#include <string>
#include <vector>
#include "JointState.hpp"
#include "ModelState.hpp"


/**
 * Generate a vector of strings containing all the joint names from the
 * JointStates associated with this model.
 *
 * @return Vector of joint names.
 */
std::vector<std::string> ModelState::getJointNames() {
  int size = jointStates.size();
  std::vector<std::string> names(size);

  for (int i = 0; i < size; i++) {
    names[i] = jointStates[i].name;
  }

  return names;
}
