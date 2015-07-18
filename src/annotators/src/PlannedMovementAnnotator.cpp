//////
// rta > annotators > PlannedMovementAnnotator.cpp

#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


using uima::Annotator;  // required for MAKE_AE


class PlannedMovementAnnotator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type Movement;
  uima::Type ControllerError;
  uima::Type PlannedMovement;
  uima::Type UnplannedMovement;

  uima::Feature mvNameFtr;  // Movement jointName
  uima::Feature ceNameFtr;  // ControllerError jointName
  uima::Feature pmNameFtr;  // PlannedMovement jointName
  uima::Feature umNameFtr;  // UnplannedMovement jointName

  // Configuration Parameters
  float minError;
  int minLength;


 public:
  /** Constructor */
  PlannedMovementAnnotator(void) {}


  /** Destructor */
  ~PlannedMovementAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("PlannedMovementAnnotator::initialize()");
    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types
   *   * Movement
   *   * ControllerError
   *   * PlannedMovement
   *   * UnplannedMovement
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("PlannedMovementAnnotator::typeSystemInit()");

    // Movement ***********************************************
    Movement = typeSystem.getType("Movement");
    if (!Movement.isValid()) {
      log->logError("Error getting Type object for Movement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    mvNameFtr = Movement.getFeatureByBaseName("jointName");

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

    // UnplannedMovement **************************************
    UnplannedMovement = typeSystem.getType("UnplannedMovement");
    if (!PlannedMovement.isValid()) {
      log->logError("Error getting Type object for UnplannedMovement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    umNameFtr = UnplannedMovement.getFeatureByBaseName("jointName");

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("PlannedMovementAnnotator::destroy()");
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
    log->logMessage("PlannedMovementAnnotator::process() begins");

    // Initialize required indices.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex ceIndex = cas.getAnnotationIndex(ControllerError);
    uima::ANIndex mvIndex = cas.getAnnotationIndex(Movement);
    uima::ANIterator ceIter = ceIndex.iterator();
    uima::ANIterator mvIter = mvIndex.iterator();

    uima::AnnotationFS move, error, annotation;
    uima::Type type;
    uima::Feature ftr;
    bool hasError;

    // Iterate over all Movement annotations.
    while (mvIter.isValid()) {
      move = mvIter.get();
      hasError = false;

      // For each movement check all controller errors.
      while (ceIter.isValid()) {
        error = ceIter.get();
        ceIter.moveToNext();

        if (
          move.getStringValue(mvNameFtr) == error.getStringValue(ceNameFtr) &&
          move.getBeginPosition() >= (error.getBeginPosition() - 20) &&
          move.getEndPosition() >= (error.getEndPosition() - 20)
        ) {
          hasError = true;
          break;
        }
      }
      ceIter.moveToFirst();

      if (hasError) {
        type = UnplannedMovement;
        ftr = umNameFtr;
      } else {
        type = PlannedMovement;
        ftr = pmNameFtr;
      }

      annotation = cas.createAnnotation(type,
                                        move.getBeginPosition(),
                                        move.getEndPosition());
      annotation.setStringValue(ftr, move.getStringValue(mvNameFtr));
      index.addFS(annotation);

      mvIter.moveToNext();
    }

    log->logMessage("PlannedMovementAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(PlannedMovementAnnotator);
