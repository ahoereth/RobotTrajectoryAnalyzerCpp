//////
// rta > MongoUrdf > MongoUrdf.cpp


#include <string>
#include <vector>
#include <map>
#include <iostream>  // cout, endl
#include "mongo/client/dbclient.h"
#include "urdf/model.h"
#include "utils.hpp"
#include "ModelState.hpp"
#include "JointState.hpp"
#include "MongoUrdf.hpp"


////////////////////////////////////////////////////////////////////////////////
/// Local utility functions

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


////////////////////////////////////////////////////////////////////////////////
/// Static MongoUrdf class members

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


////////////////////////////////////////////////////////////////////////////////
/// Public MongoUrdf class members

/**
 * Constructor. Initialize the mongo connection, parse the data and create
 * the URDF model.
 */
MongoUrdf::MongoUrdf(const std::string& host) : _host(host) {
  mongo::client::initialize();

  try {
    _conn.connect(_host);
  } catch (const mongo::DBException& e) {
    std::cout << "caught " + e.toString() << std::endl;
    exit(1);
  }
}


/** Destructor */
MongoUrdf::~MongoUrdf() {
  mongo::client::shutdown();
}


/**
 * Build the robot model from given database collections.
 *
 * @see http://wiki.ros.org/urdf/XML/model
 * @param  database
 * @param  linksCollection
 * @param  jointsCollection
 * @return Valid URDF robot model.
 */
const urdf::Model MongoUrdf::getModel(
  const std::string& database,
  const std::string& linksCollection,
  const std::string& jointsCollection
) {
  // Create the XML document.
  TiXmlDocument doc;
  TiXmlElement* root = new TiXmlElement("robot");
  root->SetAttribute("name", "test_robot");
  doc.LinkEndChild(root);

  // Query MongoDB and parse data.
  parseLinks(root, database, linksCollection);
  parseJoints(root, database, jointsCollection);

  // Save data and initialize printer.
  doc.SaveFile("test_robot.xml"); // For reference.
  TiXmlPrinter printer;
  doc.Accept(&printer);

  // Create the model.
  urdf::Model model;
  model.initString(printer.Str());
  return model;
}


/**
 * Generate the robot model states.
 *
 * @see http://wiki.ros.org/urdf/XML/model_state
 * @param database
 * @param collection
 * @return Vector of URDF model states.
 */
const std::vector<ModelState> MongoUrdf::getModelStates(
  const std::string& database,
  const std::string& collection
) {
  // Result vector which will grow while parsing data.
  std::vector<ModelState> modelStates;

  // Reusables.
  std::auto_ptr<mongo::DBClientCursor> cursor;
  std::vector<mongo::BSONElement> names, positions, velocities, efforts;
  mongo::BSONObj obj, header;

  // Query mongo db and parse results.
  cursor = _conn.query(database + "." + collection, mongo::BSONObj());
  while (cursor->more()) {
    obj = cursor->next();
    header = obj.getObjectField("header");

    ModelState modelState =
      ModelState("pr2", header.getField("stamp").Date().asInt64());

    names      = obj.getField("name").Array();
    positions  = obj.getField("position").Array();
    velocities = obj.getField("velocity").Array();
    efforts    = obj.getField("effort").Array();

    for (std::size_t i = 0, size = names.size(); i < size; ++i) {
      JointState jointState = JointState(names[i].String());
      jointState.position = positions[i].Double();
      jointState.velocity = velocities[i].Double();
      jointState.effort   = efforts[i].Double();
      // obj.getField("frame_id");  // ?
      modelState.addJointState(jointState);
    }

    modelStates.push_back(modelState);
  }

  return modelStates;
}


/**
 * Parse a specific database collection for controller input states and
 * generate a map of ModelState vectors representing it.
 *
 * @param  database
 * @param  collection
 * @return A map containing the keys `desired`, `actual` and `error` each
 *         holding a vector of ModelState objects.
 */
const std::vector< std::map<std::string, ModelState> >
MongoUrdf::getControllerStates(
  const std::string& database,
  const std::string& collection
) {
  std::vector< std::map<std::string, ModelState> > statesMapVector;
  std::auto_ptr<mongo::DBClientCursor> cursor;
  mongo::BSONObj obj;

  cursor = _conn.query(database + "." + collection, mongo::BSONObj());
  while (cursor->more()) {
    obj = cursor->next();
    std::map<std::string, ModelState> statesMap;
    statesMap["desired"] = parseControllerState(collection, obj, "desired");
    statesMap["actual"] = parseControllerState(collection, obj, "actual");
    statesMap["error"] = parseControllerState(collection, obj, "error");
    statesMapVector.push_back(statesMap);
  }

  return statesMapVector;
}


////////////////////////////////////////////////////////////////////////////////
/// Private MongoUrdf class members

/**
 * Parse the links mongo collection into URDF link elements.
 *
 * @param root The document root XML element to insert the link elements into.
 */
void MongoUrdf::parseLinks(
  TiXmlElement* root,
  const std::string& database,
  const std::string& collection
) {
  std::auto_ptr<mongo::DBClientCursor> cursor;
  cursor = _conn.query(database + "." + collection, mongo::BSONObj());

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
void MongoUrdf::parseJoints(
  TiXmlElement* root,
  const std::string& database,
  const std::string& collection
) {
  std::auto_ptr<mongo::DBClientCursor> cursor;
  cursor = _conn.query(database + "." + collection, mongo::BSONObj());

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
 * Parse one of the specific controller model states from the Mongo Database.
 *
 * @param  name      The controller name.
 * @param  obj       The database object containing the names, group and
 *                   header fields.
 * @param  groupName The group to parse: `desired`, `actual` or `error`.
 * @return A vector of ModelState objects.
 */
ModelState MongoUrdf::parseControllerState(
  const std::string& name,
  const mongo::BSONObj& obj,
  const std::string& groupName
) {
  mongo::BSONObj group  = obj.getObjectField(groupName);

  std::vector<mongo::BSONElement> names = obj.getField("joint_names").Array();
  std::vector<mongo::BSONElement> positions, velocities, effort, accelerations;
  positions     = group.getField("positions").Array();
  velocities    = group.getField("velocities").Array();
  effort        = group.getField("effort").Array();
  accelerations = group.getField("accelerations").Array();

  long stamp = obj.getObjectField("header").getField("stamp").Date().asInt64();
  ModelState modelState = ModelState(name, stamp);
  for (std::size_t i = 0; i < names.size(); ++i) {
    JointState jointState = JointState(names[i].String());
    if (i < positions.size()) {
      jointState.position = positions[i].Double();
    }

    if (i < velocities.size()) {
      jointState.velocity = velocities[i].Double();
    }

    if (i < effort.size()) {
      jointState.effort = effort[i].Double();
    }

    if (i < accelerations.size()) {
      jointState.acceleration = accelerations[i].Double();
    }

    modelState.addJointState(jointState);
  }

  return modelState;
}
