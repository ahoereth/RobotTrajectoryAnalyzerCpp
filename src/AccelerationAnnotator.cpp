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
    log->logMessage("AccelerationAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(AccelerationAnnotator);
