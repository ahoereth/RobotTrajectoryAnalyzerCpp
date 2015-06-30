/**
 * src/JointStatePopulator.cpp
 *
 * Copyright 2015 Alexander Hoereth
 */

#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>
#include "uima/api.hpp"
#include "mongo/client/dbclient.h"
#include "unicode/stringpiece.h"
#include "unicode/unistr.h"

//  using namespace uima;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using uima::Annotator;  // required for MAKE_AE


/**
 * Convert a C++ standard string to a ICU Unicode String as required by many
 * UIMA applications.
 *
 * @param  {std::string} str
 * @return {icu::UnicodeString}
 */
icu::UnicodeString sToUs(const std::string& str) {
  return icu::UnicodeString(str.data(), str.length(), US_INV);
}


/**
 * Convert a mongo BSON element to a UIMA CAS String Array Feature Structure.
 *
 * @param  {mongo::BSONElement}  field
 * @param  {uima::CAS}           cas
 * @return {uima::StringArrayFS}
 */
uima::DoubleArrayFS fieldToDoubleArrayFS(
  const mongo::BSONElement& field,
  uima::CAS& cas
) {
  std::vector<mongo::BSONElement> vec = field.Array();
  uima::DoubleArrayFS fs = cas.createDoubleArrayFS(vec.size());

  for (std::size_t i = 0; i < vec.size(); ++i) {
    fs.set(i, vec[i].Double());
  }

  return fs;
}


/**
 * Convert a mongo BSON element to a UIMA CAS Double Array Feature Structure.
 *
 * @param  {mongo::BSONElement}  field
 * @param  {uima::CAS}           cas
 * @return {uima::StringDoubleFS}
 */
uima::StringArrayFS fieldToStringArrayFS(
  const mongo::BSONElement& field,
  uima::CAS& cas
) {
  std::vector<mongo::BSONElement> vec = field.Array();
  uima::StringArrayFS fs = cas.createStringArrayFS(vec.size());

  for (std::size_t i = 0; i < vec.size(); ++i) {
    fs.set(i, sToUs(vec[i].String()));
  }

  return fs;
}


class JointStatePopulator : public Annotator {
 private:
  uima::Type JointState;
  uima::Type JointTrajectoryPoint;

  string host;
  string database;
  string collection;

  mongo::DBClientConnection conn;


 public:
  JointStatePopulator(void) {
    cout << "JointStatePopulator: Constructor" << endl;
    mongo::client::initialize();
  }

  ~JointStatePopulator(void) {
    cout << "JointStatePopulator: Destructor" << endl;
  }

  /**
   * Annotator initialization.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext &rclAnnotatorContext) {
    cout << "JointStatePopulator: initialize()" << endl;

    host = "localhost";
    if (rclAnnotatorContext.isParameterDefined("Host")) {
      rclAnnotatorContext.extractValue("Host", host);
    }

    database = "dummy1";
    if (rclAnnotatorContext.isParameterDefined("Database")) {
      rclAnnotatorContext.extractValue("Database", database);
    }

    collection = "joint_states";
    if (rclAnnotatorContext.isParameterDefined("Collection")) {
      rclAnnotatorContext.extractValue("Collection", collection);
    }

    // Log configuration info.
    rclAnnotatorContext.getLogger().logMessage(
      "Host = '" + host + "', "
      "Database = '" + database + "', "
      "Collection = '" + collection + "'");

    cout << "host: " << host << ", database: " << database
              << ", collection: " << collection << endl;

    try {
      conn.connect(host);
      cout << "connected ok" << endl;
    } catch (const mongo::DBException &e) {
      cout << "caught " << e.what() << endl;
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    return (uima::TyErrorId)UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types:
   *   * JointState
   *   * JointTrajectoryPoint
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem &crTypeSystem) {
    cout << "JointStatePopulator:: typeSystemInit() begins" << endl;

    // JointState *********************************************
    JointState = crTypeSystem.getType("JointState");
    if (!JointState.isValid()) {
      getAnnotatorContext().getLogger().logError(
        "Error getting Type object for JointState");
      cout << "JointStatePopulator::typeSystemInit - Error" << endl;
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = crTypeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      getAnnotatorContext().getLogger().logError(
        "Error getting Type object for JointTrajectoryPoint");
      cout << "JointStatePopulator::typeSystemInit - Error" << endl;
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    cout << "JointStatePopulator:: typeSystemInit() ends" << endl;

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   */
  uima::TyErrorId destroy() {
    cout << "JointStatePopulator: destroy()" << endl;
    return (uima::TyErrorId)UIMA_ERR_NONE;
  }


  /**
   * Do some work: Data processing.
   */
  uima::TyErrorId process(
    uima::CAS& cas,
    const uima::ResultSpecification& crResultSpecification
  ) {
    cout << "JointStatePopulator::process() begins" << endl;

    std::auto_ptr<mongo::DBClientCursor> cursor =
      conn.query(database + "." + collection, mongo::BSONObj());

    uima::ListFS jointStates = cas.createListFS();

    while (cursor->more()) {
      mongo::BSONObj obj = cursor->next();
      mongo::BSONObj header = obj.getObjectField("header");

      uima::FeatureStructure js = cas.createFS(JointState);
      uima::FeatureStructure jtp = cas.createFS(JointTrajectoryPoint);

      js.setFSValue(JointState.getFeatureByBaseName("jointTrajectoryPoint"),
        jtp);
      js.setIntValue(JointState.getFeatureByBaseName("seq"),
        header.getIntField("seq"));
      js.setIntValue(JointState.getFeatureByBaseName("time"),
        header.getField("stamp").Date().asInt64());
      js.setStringValue(JointState.getFeatureByBaseName("frameID"),
        sToUs(obj.getField("frame_id")));
      // TODO(ahoereth): Why not `names` like `positions` and `velocities`?
      js.setFSValue(JointState.getFeatureByBaseName("name"),
        fieldToStringArrayFS(obj.getField("name"), cas));
      jtp.setFSValue(JointTrajectoryPoint.getFeatureByBaseName("positions"),
        fieldToDoubleArrayFS(obj.getField("position"), cas));
      // TODO(ahoereth): Why not `efforts` like `positions` and `velocities`?
      jtp.setFSValue(JointTrajectoryPoint.getFeatureByBaseName("effort"),
        fieldToDoubleArrayFS(obj.getField("effort"), cas));
      jtp.setFSValue(JointTrajectoryPoint.getFeatureByBaseName("velocities"),
        fieldToDoubleArrayFS(obj.getField("velocity"), cas));

      jointStates.addLast(js);
    }

    cout << "JointStatePopulator::process() ends" << endl;
    return (uima::TyErrorId)UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(JointStatePopulator);
