//////
// rta > annotators > UnplannedStopAnnotator.cpp

#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


using uima::Annotator;  // required for MAKE_AE


class UnplannedStopAnnotator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type ControllerError;
  uima::Type PlannedMovement;
  uima::Type UnplannedStop;

  uima::Feature ceNameFtr;  // Controller Error Joint Name
  uima::Feature pmNameFtr;  // Planned Movement Joint Name
  uima::Feature usNameFtr;  // Unplanned Stop Joint Name

  // Configuration Parameters
  int threshold;
  std::vector<std::string> controllers;


 public:
  /** Constructor */
  UnplannedStopAnnotator(void) {}


  /** Destructor */
  ~UnplannedStopAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("UnplannedStopAnnotator::initialize()");

    // Threshold **********************************************
    threshold = 15;
    if (annotatorContext.isParameterDefined("Threshold")) {
      annotatorContext.extractValue("Threshold", threshold);
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

    log->logMessage("Threshold: " + utils::toString(threshold) + ", "
                    "Controllers: " + utils::join(controllers, ", "));

    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types
   *   * ControllerError
   *   * PlannedMovement
   *   * UnplannedStop
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("UnplannedStopAnnotator::typeSystemInit() begins");

    // ControllerError ****************************************
    ControllerError = typeSystem.getType("ControllerError");
    if (!ControllerError.isValid()) {
      log->logError("Error getting Type object for ControllerError");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    ceNameFtr = ControllerError.getFeatureByBaseName("jointName");

    // PlannedMovement ****************************************
    PlannedMovement = typeSystem.getType("PlannedMovement");
    if (!PlannedMovement.isValid()) {
      log->logError("Error getting Type object for PlannedMovement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    pmNameFtr = PlannedMovement.getFeatureByBaseName("jointName");

    // UnplannedStop **************************************
    UnplannedStop = typeSystem.getType("UnplannedStop");
    if (!PlannedMovement.isValid()) {
      log->logError("Error getting Type object for UnplannedStop");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    usNameFtr = UnplannedStop.getFeatureByBaseName("jointName");

    log->logMessage("UnplannedStopAnnotator::typeSystemInit() ends");
    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("UnplannedStopAnnotator::destroy()");
    return UIMA_ERR_NONE;
  }


  /**
   * Similar to the annotator's `process` function but trimmed to just
   * handle a specific controller.
   *
   * @param  cas        The current common analysis system.
   * @param  controller The controller name.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId processController(
    uima::CAS& cas,
    const std::string& controller
  ) {
    log->logMessage("UnplannedStopAnnotator::processController() begins");

    // Initialize required indices.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex ceIndex = cas.getAnnotationIndex(ControllerError);
    uima::ANIndex pmIndex = cas.getAnnotationIndex(PlannedMovement);
    uima::ANIterator ceIter = ceIndex.iterator();
    uima::ANIterator pmIter = pmIndex.iterator();

    uima::AnnotationFS error, move, annotation;
    std::size_t begin, end;
    bool stop;

    while (ceIter.isValid()) {
      error = ceIter.get();
      end = error.getEndPosition();
      stop = false;

      // For each error check every planned movement.
      while (pmIter.isValid()) {
        move = pmIter.get();

        // Are both annotations are related to the same joint?
        if (move.getStringValue(pmNameFtr) == error.getStringValue(ceNameFtr)) {
          // Does the movement end and the error start correlate?
          if (
            error.getBeginPosition() >= (move.getEndPosition() - threshold) &&
            error.getBeginPosition() <= (move.getEndPosition() + threshold)
          ) {
            begin = error.getBeginPosition();
            stop = true;
          }

          // Check if movement started while error persisted.
          if (
            move.getBeginPosition() >= (error.getBeginPosition() + threshold) &&
            move.getEndPosition() < error.getEndPosition() &&
            move.getBeginPosition() > end
          ) {
            end = move.getBeginPosition();
          }
        }

        pmIter.moveToNext();
      }
      pmIter.moveToFirst();

      if (stop) {
        annotation = cas.createAnnotation(UnplannedStop, begin, end);
        annotation.setStringValue(usNameFtr, error.getStringValue(ceNameFtr));
        index.addFS(annotation);
      }

      ceIter.moveToNext();
    }

    log->logMessage("UnplannedStopAnnotator::processController() ends");
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
    log->logMessage("UnplannedStopAnnotator::process() begins");

    uima::TyErrorId errorId = UIMA_ERR_NONE;
    for (int i = 0, size = controllers.size(); i < size; i++) {
      errorId = processController(cas,  controllers[i]);
      if (UIMA_ERR_NONE != errorId) {
        break;
      }
    }

    log->logMessage("UnplannedStopAnnotator::process() ends");
    return errorId;
  }
};


MAKE_AE(UnplannedStopAnnotator);
