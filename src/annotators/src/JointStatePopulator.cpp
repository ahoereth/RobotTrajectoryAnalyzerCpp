//////
// rta > annotators > JointStatePopulator.cpp

#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "mongo/client/dbclient.h"
#include "uimautils.hpp"


using uima::Annotator;  // required for MAKE_AE


class JointStatePopulator : public Annotator {
 private:
  mongo::DBClientConnection conn;
  uima::LogFacility* log;
  uima::CAS* currentCas;

  uima::Type JointState;
  uima::Type JointTrajectoryPoint;

  uima::Feature jsJtpFtr;    // JointState jointTrajectoryPoint
  uima::Feature jsSeqFtr;    // JointState seq
  uima::Feature jsTimeFtr;   // JointState time
  uima::Feature jsFrameFtr;  // JointState frameID
  uima::Feature jsNameFtr;   // JointState jointNames
  uima::Feature jtpPosFtr;   // JointTrajectoryPoint positions
  uima::Feature jtpEffFtr;   // JointTrajectoryPoint efforts
  uima::Feature jtpVelFtr;   // JointTrajectoryPoint velocities

  // Configuration Parameters
  std::string host;
  std::string database;
  std::string collection;


  /**
   * Convert a mongo BSON element to a UIMA CAS Double Array Feature Structure.
   *
   * TODO: Make this an external utility function.
   *
   * @see    JointStatePopulator::toStringArrayFS
   * @param  field Element containing an array of elements with string values.
   * @return Returns a string array feature structure.
   */
  uima::StringArrayFS toStringArrayFS(const mongo::BSONElement& field) {
    std::vector<mongo::BSONElement> vec = field.Array();
    uima::StringArrayFS fs = currentCas->createStringArrayFS(vec.size());

    for (std::size_t i = 0; i < vec.size(); ++i) {
      fs.set(i, utils::toUS(vec[i].String()));
    }

    return fs;
  }


  /**
   * Convert a mongo BSON element to a UIMA CAS String Array Feature Structure.
   *
   * TODO: Make this an external utility function.
   *
   * @see    JointStatePopulator::toDoubleArrayFS
   * @param  field Element containing an array of elements with double values.
   * @return Returns a double array feature structure.
   */
  uima::DoubleArrayFS toDoubleArrayFS(const mongo::BSONElement& field) {
    std::vector<mongo::BSONElement> vec = field.Array();
    uima::DoubleArrayFS fs = currentCas->createDoubleArrayFS(vec.size());

    for (std::size_t i = 0; i < vec.size(); ++i) {
      fs.set(i, vec[i].Double());
    }

    return fs;
  }


 public:
  /** Constructor */
  JointStatePopulator(void) {
    std::cout << "JointStatePopulator - mongo initialization" << std::endl;
    mongo::client::initialize();
  }


  /** Destructor */
  ~JointStatePopulator(void) {
    std::cout << "JointStatePopulator - mongo shutdown" << std::endl;
    mongo::client::shutdown();
  }


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("JointStatePopulator::initialize()");

    // Host ***************************************************
    host = "localhost";
    if (annotatorContext.isParameterDefined("Host")) {
      annotatorContext.extractValue("Host", host);
    }

    // Database ***********************************************
    database = "dummy1";
    if (annotatorContext.isParameterDefined("Database")) {
      annotatorContext.extractValue("Database", database);
    }

    // Collection *********************************************
    collection = "joint_states";
    if (annotatorContext.isParameterDefined("Collection")) {
      annotatorContext.extractValue("Collection", collection);
    }

    log->logMessage("Host: " + host + ", "
                    "Database: " + database + ", "
                    "Collection: " + collection);

    try {
      conn.connect(host);
      log->logMessage("Mongo connection established.");
    } catch (const mongo::DBException& e) {
      log->logError("caught " + e.toString());
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types:
   *   * JointState
   *   * JointTrajectoryPoint
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("JointStatePopulator::typeSystemInit()");

    // JointState *********************************************
    JointState = typeSystem.getType("JointState");
    if (!JointState.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jsJtpFtr   = JointState.getFeatureByBaseName("jointTrajectoryPoint");
    jsSeqFtr   = JointState.getFeatureByBaseName("seq");
    jsTimeFtr  = JointState.getFeatureByBaseName("time");
    jsFrameFtr = JointState.getFeatureByBaseName("frameID");
    jsNameFtr  = JointState.getFeatureByBaseName("jointNames");

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jtpPosFtr = JointTrajectoryPoint.getFeatureByBaseName("positions");
    jtpEffFtr = JointTrajectoryPoint.getFeatureByBaseName("efforts");
    jtpVelFtr = JointTrajectoryPoint.getFeatureByBaseName("velocities");

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("JointStatePopulator::destroy()");
    return UIMA_ERR_NONE;
  }


  /**
   * Data processing.
   *
   * @param  cas                 The current common analysis system.
   * @param  resultSpecification The specification of expected results as given
   *                             by the annotator descriptor.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId process(
    uima::CAS& cas,
    const uima::ResultSpecification& resultSpecification
  ) {
    log->logMessage("JointStatePopulator::process() begins");

    // Save cas for use in other member functions.
    currentCas = &cas;

    // Initialize feature structure index and reused variables.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::AnnotationFS js;
    uima::FeatureStructure jtp;
    std::size_t seq;
    mongo::BSONObj obj, header;

    // Query MongoDB.
    std::auto_ptr<mongo::DBClientCursor> cursor =
      conn.query(database + "." + collection, mongo::BSONObj());

    // Parse mongo query results.
    while (cursor->more()) {
      obj = cursor->next();
      header = obj.getObjectField("header");

      seq = header.getIntField("seq");
      js = cas.createAnnotation(JointState, seq, seq);
      jtp = cas.createFS(JointTrajectoryPoint);

      js.setFSValue(jsJtpFtr, jtp);
      js.setIntValue(jsSeqFtr, seq);
      js.setIntValue(jsTimeFtr, header.getField("stamp").Date().asInt64());
      js.setStringValue(jsFrameFtr, utils::toUS(obj.getField("frame_id")));
      js.setFSValue(jsNameFtr, toStringArrayFS(obj.getField("name")));
      jtp.setFSValue(jtpPosFtr, toDoubleArrayFS(obj.getField("position")));
      jtp.setFSValue(jtpEffFtr, toDoubleArrayFS(obj.getField("effort")));
      jtp.setFSValue(jtpVelFtr, toDoubleArrayFS(obj.getField("velocity")));

      index.addFS(js);
    }

    log->logMessage("JointStatePopulator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(JointStatePopulator);
