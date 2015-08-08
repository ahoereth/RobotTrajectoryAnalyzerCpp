//////
// rta > MongoUrdf > MongoUrdf.hpp

#ifndef __MONGOURDF_INCLUDE__
#define __MONGOURDF_INCLUDE__


#include <string>
#include <vector>
#include <map>
#include <iostream>  // cout, endl
#include "mongo/client/dbclient.h"
#include "./tinyxml.h"
#include "urdf/model.h"
#include "ModelState.hpp"
#include "JointState.hpp"


class MongoUrdf {
 public:
  static std::vector<double> getPosition(boost::shared_ptr<urdf::Link> link);
  static std::vector<double> getRotation(boost::shared_ptr<urdf::Link> link);

  explicit MongoUrdf(const std::string& host);
  ~MongoUrdf(void);
  const urdf::Model getModel(
    const std::string& database = "pr2",
    const std::string& linksCollection = "robot_links",
    const std::string& jointsCollection = "robot_joints");
  const std::vector<ModelState> getModelStates(
    const std::string& database,
    const std::string& collection = "joint_states");
  const std::vector< std::map<std::string, ModelState> > getControllerStates(
    const std::string& database,
    const std::string& collection);

 private:
  mongo::DBClientConnection _conn;
  std::string _host;
  void parseLinks(
    TiXmlElement* root,
    const std::string& database,
    const std::string& collection);
  void parseJoints(
    TiXmlElement* root,
    const std::string& database,
    const std::string& collection);
  ModelState parseControllerState(
    const std::string& name,
    const mongo::BSONObj& obj,
    const std::string& groupName);
};


#endif  // ifndef __MONGOURDF_INCLUDE__
