/**
 * src/AccelerationAnnotator.cpp
 *
 * Copyright 2015 Alexander Hoereth
 */

#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>
#include "uima/api.hpp"


using std::string;
using std::vector;
using uima::Annotator;  // required for MAKE_AE
using uima::Feature;


class AccelerationAnnotator : public Annotator {
 private:
  uima::LogFacility* log;
  uima::Type JointState;
  uima::Type JointTrajectoryPoint;
  uima::Type Acceleration;

 public:
  /** Constructor */
  AccelerationAnnotator(void) {}


  /** Destructor */
  ~AccelerationAnnotator(void) {}


  /**
   * Annotator initialization.
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
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("AccelerationAnnotator::typeSystemInit()");

    // JointState *********************************************
    JointState = typeSystem.getType("JointState");
    if (!JointState.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    // Acceleration *******************************************
    Acceleration = typeSystem.getType("Acceleration");
    if (!Acceleration.isValid()) {
      log->logError("Error getting Type object for Acceleration");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   */
  uima::TyErrorId destroy() {
    log->logMessage("AccelerationAnnotator::destroy()");
    return UIMA_ERR_NONE;
  }


  /**
   * Data processing.
   */
  uima::TyErrorId process(
    uima::CAS& cas,
    const uima::ResultSpecification& resultSpecification
  ) {
    log->logMessage("AccelerationAnnotator::process() begins");

    // Initialize features.
    Feature timeFtr = JointState.getFeatureByBaseName("time");
    Feature jtpFtr = JointState.getFeatureByBaseName("jointTrajectoryPoint");
    Feature jtpVelFtr = JointTrajectoryPoint.getFeatureByBaseName("velocities");
    Feature accVelFtr = Acceleration.getFeatureByBaseName("value");

    // Intialize indices and the index iterator.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex jsIndex = cas.getAnnotationIndex(JointState);
    uima::ANIterator jsIter = jsIndex.iterator();

    // Initialize reused variables for the loop.
    uima::ListFS accelerations = cas.createListFS();
    uima::FeatureStructure js, jtp;
    uima::DoubleArrayFS velocitiesPrev, velocitiesNext, values;
    int timePrev, timeNext;
    double velocityDiff, timeDiff, accelerationValue;

    if (jsIter.isValid()) {
      // Get initial values.
      js = jsIter.get();
      jtp = js.getFSValue(jtpFtr);
      timePrev = js.getIntValue(timeFtr);
      velocitiesPrev = jtp.getDoubleArrayFSValue(jtpVelFtr);

      // Loop through all consecutive joint states.
      jsIter.moveToNext();
      while (jsIter.isValid()) {
        // Get consecutive values.
        js = jsIter.get();
        jtp = js.getFSValue(jtpFtr);
        timeNext = js.getIntValue(timeFtr);
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
        uima::FeatureStructure acceleration = cas.createFS(Acceleration);
        acceleration.setFSValue(accVelFtr, values);
        accelerations.addLast(acceleration);
        index.addFS(acceleration);

        // Remember values and move to next joint state.
        velocitiesPrev = jtp.getDoubleArrayFSValue(jtpVelFtr);
        timePrev = js.getIntValue(timeFtr);
        jsIter.moveToNext();
      }
    }

    log->logMessage("AccelerationAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(AccelerationAnnotator);
