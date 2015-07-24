//////
// rta > MongoUrdf > MongoUrdf.cpp


#include <string>
#include <vector>
#include "boost/smart_ptr.hpp"
#include "mongo/client/dbclient.h"
#include "urdf/model.h"
#include "utils.hpp"
#include "MongoUrdf.hpp"


/**
 * Generates a space seperated concatenation of coordinates given as a
 * BSONElement.
 *
 * @param  field Mongo source field.
 * @return Resulting string.
 */
std::string mongoCoordinatesToString(
  mongo::BSONElement field
) {
  std::vector<mongo::BSONElement> data = field.Array();
  std::string value = "";

  for (
    std::vector<mongo::BSONElement>::iterator it = data.begin();
    it != data.end();
    ++it
  ) {
    if (value.compare("") != 0) { value += " "; }
    value += it->toString(false);
  }

  return value;
}


/** Destructor */
MongoUrdf::~MongoUrdf() {
  mongo::client::shutdown();
}


/**
 * Constructor. Initialize the mongo connection, parse the data and create
 * the URDF model.
 */
MongoUrdf::MongoUrdf(
  const std::string& host,
  const std::string& database,
  const std::string& linksCollection,
  const std::string& jointsCollection
) : _host(host),
    _database(database),
    _linksCollection(linksCollection),
    _jointsCollection(jointsCollection) {
  mongo::client::initialize();

  try {
    conn.connect(_host);

    // Create the XML document.
    TiXmlDocument doc;
    TiXmlElement* root = new TiXmlElement("robot");
    root->SetAttribute("name", "test_robot");
    doc.LinkEndChild(root);
    parseLinks(root);
    parseJoints(root);
    doc.SaveFile("test_robot.xml"); // For reference.
    TiXmlPrinter printer;
    doc.Accept(&printer);

    // Create the model.
    _model.initString(printer.Str());
  } catch (const mongo::DBException& e) {
    std::cout << "caught " + e.toString() << std::endl;
    exit(1);
  }
}


/**
 * Parse the links mongo collection into URDF link elements.
 *
 * @param root The document root XML element to insert the link elements into.
 */
void MongoUrdf::parseLinks(TiXmlElement* root) {
  std::auto_ptr<mongo::DBClientCursor> cursor =
    conn.query(_database + "." + _linksCollection, mongo::BSONObj());

  mongo::BSONObj obj;

  while (cursor->more()) {
    obj = cursor->next();

    TiXmlElement* link = new TiXmlElement("link");
    link->SetAttribute("name", obj.getStringField("name"));
    root->LinkEndChild(link);

    TiXmlElement* inertial = new TiXmlElement("inertial");
    link->LinkEndChild(inertial);

    TiXmlElement* mass = new TiXmlElement("mass");
    mass->SetAttribute("value", obj.getIntField("mass"));
    inertial->LinkEndChild(mass);

    TiXmlElement* origin = new TiXmlElement("origin");
    origin->SetAttribute("xyz", mongoCoordinatesToString(obj.getField("xyz")));
    origin->SetAttribute("rpy", mongoCoordinatesToString(obj.getField("rpy")));
    inertial->LinkEndChild(origin);

    // TODO: The following inertia matrix is made up.
    // "The 3x3 rotational inertia matrix, represented in the inertia frame."
    TiXmlElement* inertia = new TiXmlElement("inertia");
    inertia->SetAttribute("ixx", "1.0");
    inertia->SetAttribute("ixy", "0.0");
    inertia->SetAttribute("ixz", "0.0");
    inertia->SetAttribute("iyy", "1.0");
    inertia->SetAttribute("iyz", "0.0");
    inertia->SetAttribute("izz", "1.0");
    inertial->LinkEndChild(inertia);
  }
}


/**
 * Parse the joints mongo collection into URDF joint elements.
 *
 * @param root The document root XML element to insert the joint elements into.
 */
void MongoUrdf::parseJoints(TiXmlElement* root) {
  std::auto_ptr<mongo::DBClientCursor> cursor =
    conn.query(_database + "." + _jointsCollection, mongo::BSONObj());

  mongo::BSONObj obj;

  while (cursor->more()) {
    obj = cursor->next();

    TiXmlElement* joint = new TiXmlElement("joint");
    joint->SetAttribute("name", obj.getStringField("name"));
    joint->SetAttribute("type", utils::toLower(obj.getStringField("type")));
    root->LinkEndChild(joint);

    TiXmlElement* child = new TiXmlElement("child");
    child->SetAttribute("link", obj.getStringField("child"));
    joint->LinkEndChild(child);

    TiXmlElement* parent = new TiXmlElement("parent");
    parent->SetAttribute("link", obj.getStringField("parent"));
    joint->LinkEndChild(parent);

    TiXmlElement* origin = new TiXmlElement("origin");
    origin->SetAttribute("xyz", mongoCoordinatesToString(obj.getField("xyz")));
    origin->SetAttribute("rpy", mongoCoordinatesToString(obj.getField("rpy")));
    joint->LinkEndChild(origin);

    TiXmlElement* limit = new TiXmlElement("limit");
    limit->SetAttribute("upper", obj.getIntField("upper-limit"));
    limit->SetAttribute("lower", obj.getIntField("lower-limit"));
    limit->SetAttribute("effort", obj.getIntField("effort-limit"));
    limit->SetAttribute("velocity", obj.getIntField("velocity-limit"));
    joint->LinkEndChild(limit);

    TiXmlElement* axis = new TiXmlElement("axis");
    axis->SetAttribute("xyz", mongoCoordinatesToString(obj.getField("axis")));
    joint->LinkEndChild(axis);
  }
}


/**
 * Utility function for extracting the inertial origin position information
 * from a link into an easy to work with vector.
 *
 * @param link Source URDF link element.
 * @param Result verctor.
 */
std::vector<double> MongoUrdf::getPosition(boost::shared_ptr<urdf::Link> link) {
  urdf::Vector3 pos = link->inertial->origin.position;
  std::vector<double> vec(3);
  vec[0] = pos.x;
  vec[1] = pos.y;
  vec[2] = pos.z;
  return vec;
}


/**
 * Utility function for extracting the inertial origin rotation information
 * from a link into an easy to work with vector.
 *
 * @param link Source URDF link element.
 * @param Result verctor.
 */
std::vector<double> MongoUrdf::getRotation(boost::shared_ptr<urdf::Link> link) {
  urdf::Rotation rotation = link->inertial->origin.rotation;
  std::vector<double> vec(3);
  rotation.getRPY(vec[0], vec[1], vec[2]);
  return vec;
}
