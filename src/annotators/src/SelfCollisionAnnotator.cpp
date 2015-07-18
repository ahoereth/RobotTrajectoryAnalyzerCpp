//////
// rta > annotators > SelfCollisionAnnotator.cpp

#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


using uima::Annotator;  // required for MAKE_AE


class SelfCollisionAnnotator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type UnplannedMovement;
  uima::Type UnplannedStop;
  uima::Type SelfCollision;

  uima::Feature umNameFtr;  // Unplanned Movement Joint Name
  uima::Feature usNameFtr;  // Unplanned Stop Joint Name
  uima::Feature scVicFtr;   // Self Collision Victim
  uima::Feature scPerFtr;   // Self Collision Perpetrator

  // Configuration Parameters
  int threshold;


 public:
  /** Constructor */
  SelfCollisionAnnotator(void) {}


  /** Destructor */
  ~SelfCollisionAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("SelfCollisionAnnotator::initialize()");

    // Threshold **********************************************
    threshold = 15;
    if (annotatorContext.isParameterDefined("Threshold")) {
      annotatorContext.extractValue("Threshold", threshold);
    }

    log->logMessage("Threshold: " + utils::toString(threshold));
    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types
   *   * UnplannedMovement
   *   * UnplannedStop
   *   * SelfCollision
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("SelfCollisionAnnotator::typeSystemInit()");

    // UnplannedMovement **********************************
    UnplannedMovement = typeSystem.getType("UnplannedMovement");
    if (!UnplannedMovement.isValid()) {
      log->logError("Error getting Type object for UnplannedMovement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    umNameFtr = UnplannedMovement.getFeatureByBaseName("jointName");

    // UnplannedStop **************************************
    UnplannedStop = typeSystem.getType("UnplannedStop");
    if (!UnplannedStop.isValid()) {
      log->logError("Error getting Type object for UnplannedStop");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    usNameFtr = UnplannedStop.getFeatureByBaseName("jointName");

    // SelfCollision **************************************
    SelfCollision = typeSystem.getType("SelfCollision");
    if (!SelfCollision.isValid()) {
      log->logError("Error getting Type object for SelfCollision");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    scVicFtr = SelfCollision.getFeatureByBaseName("victim");
    scPerFtr = SelfCollision.getFeatureByBaseName("perpetrator");

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("SelfCollisionAnnotator::destroy()");
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
    log->logMessage("SelfCollisionAnnotator::process() begins");

    // Initialize required indices, iterators and reused variables.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex usIndex = cas.getAnnotationIndex(UnplannedStop);
    uima::ANIndex umIndex = cas.getAnnotationIndex(UnplannedMovement);
    uima::ANIterator usIter = usIndex.iterator();
    uima::ANIterator umIter = umIndex.iterator();
    uima::AnnotationFS stop, move, collision;
    uima::UnicodeStringRef victim, perpetrator;
    std::size_t begin, end;

    // Iterate over all unplanned stops.
    while (usIter.isValid()) {
      stop = usIter.get();
      perpetrator = stop.getStringValue(usNameFtr);
      begin = stop.getBeginPosition();
      end = stop.getEndPosition();

      // Check if the current unplanned stop correlates with an unplanned
      // movement of a different joint.
      umIter.moveToFirst();
      while (umIter.isValid()) {
        move = umIter.get();
        umIter.moveToNext();
        if (
          move.getStringValue(umNameFtr) != perpetrator &&
          move.getBeginPosition() >= (begin - threshold) &&
          move.getBeginPosition() <= (end + threshold)
        ) {
          victim = move.getStringValue(umNameFtr);
          break;
        }
      }

      // If there was a collission create an annotation and add it to the index.
      if (!victim.isEmpty()) {
        collision = cas.createAnnotation(SelfCollision, begin, end);
        collision.setStringValue(scPerFtr, perpetrator);
        collision.setStringValue(scVicFtr, victim);
        index.addFS(collision);
      }

      usIter.moveToNext();
    }

    log->logMessage("SelfCollisionAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(SelfCollisionAnnotator);
