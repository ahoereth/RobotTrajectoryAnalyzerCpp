//////
// rta > annotators > ControllerAnnotator.cpp

#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "mongo/client/dbclient.h"
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


using uima::Annotator;  // required for MAKE_AE


class ControllerAnnotator : public Annotator {
 private:
  uima::LogFacility* log;
  uima::CAS* currentCas;

  uima::Type JointState;
  uima::Type JointTrajectoryPoint;
  uima::Type ControllerInput;

  uima::Feature jsTimeFtr;   // Joint State Time
  uima::Feature jtpPosFtr;   // Joint Trajectory Point Positions
  uima::Feature jtpEffFtr;   // Joint Trajectory Point Effort
  uima::Feature jtpVelFtr;   // Joint Trajectory Point Velocities
  uima::Feature jtpAccFtr;   // Joint Trajectory Point Accelerations
  uima::Feature ciTypeFtr;   // Controller Input Controller Type
  uima::Feature ciTimeFtr;   // Controller Input Time
  uima::Feature ciJnsFtr;    // Controller Input Joint Names
  uima::Feature ciDesFtr;    // Controller Input Desired
  uima::Feature ciActFtr;    // Controller Input Actual
  uima::Feature ciErrFtr;    // Controller Input Error

  // Configuration Parameters
  std::string host;
  std::string database;
  std::vector<std::string> controllers;

  mongo::DBClientConnection conn;


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


  /**
   * Create a Joint Trajectory Point Feature Structure from a mongo BSON object
   * containing the fields position, effort velocity, and acelerations.
   *
   * @param  obj Source bson object.
   * @return Joint Trajectory Point with all the information from the object.
   */
  uima::FeatureStructure readJtp(const mongo::BSONObj& obj) {
    uima::FeatureStructure jtp = currentCas->createFS(JointTrajectoryPoint);
    jtp.setFSValue(jtpPosFtr, toDoubleArrayFS(obj.getField("positions")));
    jtp.setFSValue(jtpEffFtr, toDoubleArrayFS(obj.getField("effort")));
    jtp.setFSValue(jtpVelFtr, toDoubleArrayFS(obj.getField("velocities")));
    jtp.setFSValue(jtpAccFtr, toDoubleArrayFS(obj.getField("accelerations")));
    return jtp;
  }


 public:
  /** Constructor */
  ControllerAnnotator(void) {
    std::cout << "ControllerAnnotator - mongo initialization" << std::endl;
    mongo::client::initialize();
  }


  /** Destructor */
  ~ControllerAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("ControllerAnnotator::initialize()");

    host = "localhost";
    if (annotatorContext.isParameterDefined("Host")) {
      annotatorContext.extractValue("Host", host);
    }

    database = "dummy1";
    if (annotatorContext.isParameterDefined("Database")) {
      annotatorContext.extractValue("Database", database);
    }

    if (annotatorContext.isParameterDefined("Controllers")) {
      std::vector<icu::UnicodeString> tmpControllers;
      annotatorContext.extractValue("Controllers", tmpControllers);
      controllers = utils::toString(tmpControllers);
    } else {
      controllers.push_back("r_arm_controller_state");
      controllers.push_back("l_arm_controller_state");
    }

    log->logMessage("Host: " + host + ", Database: " + database + ", "
      "Controllers: " + utils::join(controllers, ", "));

    try {
      conn.connect(host);
      log->logMessage("connected ok");
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
   *   * ControllerInput
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("ControllerAnnotator::typeSystemInit() begins");

    // JointState *********************************************
    JointState = typeSystem.getType("JointState");
    if (!JointState.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jsTimeFtr  = JointState.getFeatureByBaseName("time");

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jtpPosFtr = JointTrajectoryPoint.getFeatureByBaseName("positions");
    jtpEffFtr = JointTrajectoryPoint.getFeatureByBaseName("effort");
    jtpVelFtr = JointTrajectoryPoint.getFeatureByBaseName("velocities");
    jtpAccFtr = JointTrajectoryPoint.getFeatureByBaseName("accelerations");

    // ControllerInput ****************************************
    ControllerInput = typeSystem.getType("ControllerInput");
    if (!ControllerInput.isValid()) {
      log->logError("Error getting Type object for ControllerInput");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    ciTypeFtr = ControllerInput.getFeatureByBaseName("controllerType");
    ciTimeFtr = ControllerInput.getFeatureByBaseName("time");
    ciJnsFtr  = ControllerInput.getFeatureByBaseName("jointNames");
    ciDesFtr  = ControllerInput.getFeatureByBaseName("desired");
    ciActFtr  = ControllerInput.getFeatureByBaseName("actual");
    ciErrFtr  = ControllerInput.getFeatureByBaseName("error");

    log->logMessage("ControllerAnnotator::typeSystemInit() ends");
    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("ControllerAnnotator::destroy()");
    return UIMA_ERR_NONE;
  }


  /**
   * Similar to the annotator's `process` function but trimmed to just
   * handle a specific controller.
   *
   * @param  cas        The current common analysis system.
   * @param  controller The controller name which is also equivalent to the
   *                    mongo db collection's name.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId processController(
    uima::CAS& cas,
    const std::string& controller
  ) {
    log->logMessage("ControllerAnnotator::processController() begins");

    // Initialize required indices.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex jsIndex = cas.getAnnotationIndex(JointState);
    uima::ANIterator jsIter = cas.getAnnotationIndex(JointState).iterator();

    uima::AnnotationFS ci, js;
    int stamp, begin, end;

    // Query MongoDB and parse result cursor.
    std::auto_ptr<mongo::DBClientCursor> cursor =
      conn.query(database + "." + controller, mongo::BSONObj());

    while (cursor->more()) {
      mongo::BSONObj obj = cursor->next();
      mongo::BSONObj header = obj.getObjectField("header");
      stamp = header.getField("stamp").Date().asInt64();
      begin = 0, end = 0;

      // TODO: Clarify relation between controller- and joint state
      if (
        jsIter.isValid() && jsIter.peekPrevious().isValid() &&
        stamp < (js = jsIter.get()).getIntValue(jsTimeFtr)
      ) {
        jsIter.moveToNext();
        begin = js.getBeginPosition();
        end = js.getEndPosition();
      }

      ci = cas.createAnnotation(ControllerInput, begin, end);
      ci.setStringValue(ciTypeFtr, utils::toUS(controller));
      ci.setIntValue(ciTimeFtr, stamp);
      ci.setFSValue(ciJnsFtr, toStringArrayFS(obj.getField("joint_names")));
      ci.setFSValue(ciDesFtr, readJtp(obj.getField("desired").Obj()));
      ci.setFSValue(ciActFtr, readJtp(obj.getField("actual").Obj()));
      ci.setFSValue(ciErrFtr, readJtp(obj.getField("error").Obj()));

      index.addFS(ci);
    }

    log->logMessage("ControllerAnnotator::processController() ends");
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
    log->logMessage("ControllerAnnotator::process() begins");

    // Save cas for use in other member functions.
    currentCas = &cas;

    uima::TyErrorId errorId = UIMA_ERR_NONE;
    for (int i = 0, size = controllers.size(); i < size; i++) {
      errorId = processController(cas,  controllers[i]);
      if (UIMA_ERR_NONE != errorId) {
        break;
      }
    }

    log->logMessage("ControllerAnnotator::process() ends");
    return errorId;
  }
};


MAKE_AE(ControllerAnnotator);
