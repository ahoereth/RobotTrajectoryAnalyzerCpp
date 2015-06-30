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

//  using namespace uima;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using uima::Annotator;  // required for MAKE_AE


class AccelerationAnnotator : public Annotator {
 private:
  uima::Type JointState;
  uima::Type JointTrajectoryPoint;
  uima::Type Acceleration;

 public:
  AccelerationAnnotator(void) {
    cout << "AccelerationAnnotator: Constructor" << endl;
  }

  ~AccelerationAnnotator(void) {
    cout << "AccelerationAnnotator: Destructor" << endl;
  }


  /**
   * Annotator initialization.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext &rclAnnotatorContext) {
    cout << "AccelerationAnnotator: initialize()" << endl;
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
    cout << "AccelerationAnnotator:: typeSystemInit()" << endl;

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

    // Acceleration *******************************************
    Acceleration = crTypeSystem.getType("Acceleration");
    if (!Acceleration.isValid()) {
      getAnnotatorContext().getLogger().logError(
        "Error getting Type object for Acceleration");
      cout << "JointStatePopulator::typeSystemInit - Error" << endl;
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   */
  uima::TyErrorId destroy() {
    cout << "AccelerationAnnotator: destroy()" << endl;
    return (uima::TyErrorId)UIMA_ERR_NONE;
  }


  /**
   * Do some work: Data processing.
   */
  uima::TyErrorId process(
    uima::CAS& cas,
    const uima::ResultSpecification& crResultSpecification
  ) {
    cout << "AccelerationAnnotator::process() begins" << endl;
    cout << "AccelerationAnnotator::process() ends" << endl;
    return (uima::TyErrorId)UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(AccelerationAnnotator);
