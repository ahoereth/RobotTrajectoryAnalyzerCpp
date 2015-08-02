//////
// rta > annotators > JointStatepopulator.cpp

#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "uimautils.hpp"
#include "MongoUrdf.hpp"


using uima::Annotator;  // required for MAKE_AE


class JointStatePopulator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type JointStateType;
  uima::Type JointTrajectoryPoint;

  uima::Feature jsJtpFtr;    // JointState jointTrajectoryPoint
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


 public:
  /** Constructor */
  JointStatePopulator(void) {}


  /** Destructor */
  ~JointStatePopulator(void) {}


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
    JointStateType = typeSystem.getType("JointState");
    if (!JointStateType.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jsJtpFtr   = JointStateType.getFeatureByBaseName("jointTrajectoryPoint");
    jsTimeFtr  = JointStateType.getFeatureByBaseName("time");
    jsFrameFtr = JointStateType.getFeatureByBaseName("frameID");
    jsNameFtr  = JointStateType.getFeatureByBaseName("jointNames");

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

    // Parse MongoDB data into an urdf model.
    MongoUrdf* urdf = new MongoUrdf(host);
    std::vector<ModelState> states = urdf->getModelStates(database, collection);
    delete urdf;

    // Initialize feature structure index and reused variables.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::AnnotationFS js;
    uima::FeatureStructure jtp;
    std::size_t jsCount;
    uima::StringArrayFS names;
    uima::DoubleArrayFS positions, velocities, efforts;

    for (std::size_t i = 0, size = states.size(); i < size; i++) {
      ModelState state = states[i];

      // Aggregate joint states into array feature structures.
      jsCount = state.jointStates.size();
      names      = cas.createStringArrayFS(jsCount);
      positions  = cas.createDoubleArrayFS(jsCount);
      velocities = cas.createDoubleArrayFS(jsCount);
      efforts    = cas.createDoubleArrayFS(jsCount);
      for (std::size_t j = 0; j < jsCount; j++) {
        JointState joint = state.jointStates[j];
        names.set(j, utils::toUS(joint.name));
        positions.set(j, joint.position);
        velocities.set(j, joint.velocity);
        efforts.set(j, joint.effort);
      }

      // Create Joint State and Joint Trajectory Point feature structures.
      jtp = cas.createFS(JointTrajectoryPoint);
      jtp.setFSValue(jtpPosFtr, positions);
      jtp.setFSValue(jtpEffFtr, efforts);
      jtp.setFSValue(jtpVelFtr, velocities);
      js = cas.createAnnotation(JointStateType, state.time, state.time);
      js.setFSValue(jsJtpFtr, jtp);
      js.setIntValue(jsTimeFtr, state.time);
      // js.setStringValue(jsFrameFtr, frameID);
      js.setFSValue(jsNameFtr, names);

      index.addFS(js);
    }

    log->logMessage("JointStatePopulator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(JointStatePopulator);
