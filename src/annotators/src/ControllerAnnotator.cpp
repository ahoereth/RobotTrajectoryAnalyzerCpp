//////
// rta > annotators > ControllerAnnotator.cpp

#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "uimautils.hpp"
#include "MongoUrdf.hpp"

using uima::Annotator;  // required for MAKE_AE


class ControllerAnnotator : public Annotator {
 private:
  uima::LogFacility* log;
  uima::CAS* currentCas;

  uima::Type JointStateType;
  uima::Type JointTrajectoryPoint;
  uima::Type ControllerInput;

  uima::Feature jsTimeFtr;  // JointState time
  uima::Feature jtpPosFtr;  // JointTrajectoryPoint positions
  uima::Feature jtpEffFtr;  // JointTrajectoryPoint efforts
  uima::Feature jtpVelFtr;  // JointTrajectoryPoint velocities
  uima::Feature jtpAccFtr;  // JointTrajectoryPoint accelerations
  uima::Feature ciTypeFtr;  // ControllerInput controllerType
  uima::Feature ciTimeFtr;  // ControllerInput time
  uima::Feature ciJnsFtr;   // ControllerInput jointNames
  uima::Feature ciDesFtr;   // ControllerInput desired
  uima::Feature ciActFtr;   // ControllerInput actual
  uima::Feature ciErrFtr;   // ControllerInput error

  // Configuration Parameters
  std::string host;
  std::string database;
  std::vector<std::string> controllers;


  /**
   * Create a Joint Trajectory Point Feature Structure from a vector of
   * JointState objects.
   *
   * @param  jointStates Source vector of JointState objects.
   * @return Joint Trajectory Point with all the information from the objects.
   */
  uima::FeatureStructure toJtp(const std::vector<JointState>& jointStates) {
    std::size_t size = jointStates.size(), i = 0;
    std::vector<JointState>::const_iterator it;

    uima::DoubleArrayFS positions     = currentCas->createDoubleArrayFS(size);
    uima::DoubleArrayFS effort        = currentCas->createDoubleArrayFS(size);
    uima::DoubleArrayFS velocities    = currentCas->createDoubleArrayFS(size);
    uima::DoubleArrayFS accelerations = currentCas->createDoubleArrayFS(size);
    for (it = jointStates.begin(); it != jointStates.end(); it++, i++) {
      positions.set(i, it->position);
      effort.set(i, it->effort);
      velocities.set(i, it->velocity);
      accelerations.set(i, it->acceleration);
    }

    uima::FeatureStructure jtp = currentCas->createFS(JointTrajectoryPoint);
    jtp.setFSValue(jtpPosFtr, positions);
    jtp.setFSValue(jtpEffFtr, effort);
    jtp.setFSValue(jtpVelFtr, velocities);
    jtp.setFSValue(jtpAccFtr, accelerations);

    return jtp;
  }


 public:
  /** Constructor */
  ControllerAnnotator(void) {}


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

    // Controllers ********************************************
    if (annotatorContext.isParameterDefined("Controllers")) {
      std::vector<icu::UnicodeString> tmpControllers;
      annotatorContext.extractValue("Controllers", tmpControllers);
      controllers = utils::toString(tmpControllers);
    } else {
      controllers.push_back("r_arm_controller_state");
      controllers.push_back("l_arm_controller_state");
    }

    log->logMessage("Host: " + host + ", "
                    "Database: " + database + ", "
                    "Controllers: " + utils::join(controllers, ", "));

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
    JointStateType = typeSystem.getType("JointState");
    if (!JointStateType.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jsTimeFtr  = JointStateType.getFeatureByBaseName("time");

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jtpPosFtr = JointTrajectoryPoint.getFeatureByBaseName("positions");
    jtpEffFtr = JointTrajectoryPoint.getFeatureByBaseName("efforts");
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

    // Parse MongoDB data into an urdf model.
    MongoUrdf* urdf = new MongoUrdf(host);
    typedef std::vector< std::map<std::string, ModelState> > statesT;
    statesT states = urdf->getControllerStates(database, controller);
    delete urdf;

    // Initialize required indices.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIterator jsIter = cas.getAnnotationIndex(JointStateType).iterator();
    uima::AnnotationFS ci, js;
    std::vector<std::string> names;
    int stamp, begin, end;

    for (statesT::iterator it = states.begin(); it != states.end(); it++) {
      ModelState desired = it->find("desired")->second;
      ModelState actual  = it->find("actual")->second;
      ModelState error   = it->find("error")->second;

      names = desired.getJointNames();
      stamp = desired.time;
      //begin = 0, end = 0;

      /*if (
        jsIter.isValid() && jsIter.peekPrevious().isValid() &&
        stamp < (js = jsIter.get()).getIntValue(jsTimeFtr)
      ) {
        jsIter.moveToNext();
        begin = js.getBeginPosition();
        end = js.getEndPosition();
      }*/

      ci = cas.createAnnotation(ControllerInput, stamp, stamp);
      ci.setStringValue(ciTypeFtr, utils::toUS(controller));
      ci.setIntValue(ciTimeFtr, stamp);
      ci.setFSValue(ciJnsFtr, utils::toStringArrayFS(cas, names));
      ci.setFSValue(ciDesFtr, toJtp(desired.jointStates));
      ci.setFSValue(ciActFtr, toJtp(actual.jointStates));
      ci.setFSValue(ciErrFtr, toJtp(error.jointStates));

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
