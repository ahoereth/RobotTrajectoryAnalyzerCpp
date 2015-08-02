//////
// rta > annotators > AccelerationAnnotator.cpp

#include "uima/api.hpp"


using uima::Annotator;  // required for MAKE_AE


class AccelerationAnnotator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type JointState;
  uima::Type JointTrajectoryPoint;
  uima::Type Acceleration;

  uima::Feature jsTimeFtr;  // JointState time
  uima::Feature jsJtpFtr;   // JointState jointTrajectoryPoint
  uima::Feature jtpVelFtr;  // JointTrajectoryPoint velocities
  uima::Feature accValFtr;  // Acceleration values


 public:
  /** Constructor */
  AccelerationAnnotator(void) {}


  /** Destructor */
  ~AccelerationAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("AccelerationAnnotator::initialize()");
    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types:
   *   * JointState
   *   * JointTrajectoryPoint
   *   * Acceleration
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("AccelerationAnnotator::typeSystemInit()");

    // JointState *********************************************
    JointState = typeSystem.getType("JointState");
    if (!JointState.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jsTimeFtr = JointState.getFeatureByBaseName("time");
    jsJtpFtr  = JointState.getFeatureByBaseName("jointTrajectoryPoint");

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jtpVelFtr = JointTrajectoryPoint.getFeatureByBaseName("velocities");

    // Acceleration *******************************************
    Acceleration = typeSystem.getType("Acceleration");
    if (!Acceleration.isValid()) {
      log->logError("Error getting Type object for Acceleration");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    accValFtr = Acceleration.getFeatureByBaseName("values");

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("AccelerationAnnotator::destroy()");
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
    log->logMessage("AccelerationAnnotator::process() begins");

    // Intialize indices and the index iterator.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex jsIndex = cas.getAnnotationIndex(JointState);
    uima::ANIterator jsIter = jsIndex.iterator();

    // Initialize reused variables for the loop.
    uima::AnnotationFS js, acceleration;
    uima::FeatureStructure jtp;
    uima::DoubleArrayFS velocitiesPrev, velocitiesNext, values;
    int timePrev, timeNext, pos;
    double velocityDiff, timeDiff, accelerationValue;

    if (jsIter.isValid()) {
      // Get initial values.
      js = jsIter.get();
      jtp = js.getFSValue(jsJtpFtr);
      timePrev = js.getIntValue(jsTimeFtr);
      velocitiesPrev = jtp.getDoubleArrayFSValue(jtpVelFtr);

      // Loop through all consecutive joint states.
      jsIter.moveToNext();
      while (jsIter.isValid()) {
        // Get consecutive values.
        js = jsIter.get();
        jtp = js.getFSValue(jsJtpFtr);
        timeNext = js.getIntValue(jsTimeFtr);
        velocitiesNext = jtp.getDoubleArrayFSValue(jtpVelFtr);
        values = cas.createDoubleArrayFS(velocitiesNext.size());
        // Calculate acceleration values.
        for (std::size_t i = 0; i < values.size(); ++i) {
          velocityDiff = velocitiesNext.get(i) - velocitiesPrev.get(i);
          timeDiff = timeNext - timePrev;
          accelerationValue = (0 != timeDiff ? velocityDiff / timeDiff : 0);
          values.set(i, accelerationValue);
        }

        // Create acceleration feature structure and save to annotation index.
        pos = js.getBeginPosition();
        acceleration = cas.createAnnotation(Acceleration, pos, pos);
        acceleration.setFSValue(accValFtr, values);
        index.addFS(acceleration);

        // Remember values and move to next joint state.
        velocitiesPrev = jtp.getDoubleArrayFSValue(jtpVelFtr);
        timePrev = js.getIntValue(jsTimeFtr);
        jsIter.moveToNext();
      }
    }

    log->logMessage("AccelerationAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(AccelerationAnnotator);
