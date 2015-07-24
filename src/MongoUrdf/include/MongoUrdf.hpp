//////
// rta > MongoUrdf > MongoUrdf.hpp

#ifndef __MONGOURDF_INCLUDE__
#define __MONGOURDF_INCLUDE__


#include <string>
#include <vector>
#include "boost/smart_ptr.hpp"
#include "mongo/client/dbclient.h"
#include "tinyxml.h"
#include "urdf/model.h"


class MongoUrdf {
 public:
  static std::vector<double> getPosition(boost::shared_ptr<urdf::Link> link);
  static std::vector<double> getRotation(boost::shared_ptr<urdf::Link> link);

  MongoUrdf(const std::string& host,
            const std::string& database,
            const std::string& linksCollection,
            const std::string& jointsCollection);
  ~MongoUrdf(void);
  const urdf::Model& getModel() { return _model; }


 private:
  mongo::DBClientConnection conn;
  urdf::Model _model;
  std::string _host;
  std::string _database;
  std::string _linksCollection;
  std::string _jointsCollection;
  void parseLinks(TiXmlElement* root);
  void parseJoints(TiXmlElement* root);
  std::string parse();
};


#endif  // ifndef __MONGOURDF_INCLUDE__
