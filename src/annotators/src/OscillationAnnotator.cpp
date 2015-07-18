//////
// rta > annotators > OscillationAnnotator.cpp

#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "utils.hpp"


using uima::Annotator;  // required for MAKE_AE


class OscillationAnnotator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type Movement;
  uima::Type PositiveMovement;
  uima::Type Oscillation;

  uima::Feature movJnFtr;  // Movement jointName
  uima::Feature pSpFtr;    // PositiveMovement startPosition
  uima::Feature pEpFtr;    // PositiveMovement endPosition
  uima::Feature oscJnFtr;  // Oscillation jointName

  // Configuration Parameters
  float maxTimeVariance;
  float maxPositionVariance;


 public:
  /** Constructor */
  OscillationAnnotator(void) {}


  /** Destructor */
  ~OscillationAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("OscillationAnnotator::initialize()");

    // MaxTimeVariance ****************************************
    maxTimeVariance = 50;
    if (annotatorContext.isParameterDefined("MaxTimeVariance")) {
      annotatorContext.extractValue("MaxTimeVariance", maxTimeVariance);
    }

    // MaxPositionVariance ************************************
    maxPositionVariance = 10;
    if (annotatorContext.isParameterDefined("MaxPositionVariance")) {
      annotatorContext.extractValue("MaxPositionVariance", maxPositionVariance);
    }

    log->logMessage("MaxTimeVariance: " + utils::toString(maxTimeVariance) +
      ", MaxPositionVariance: " + utils::toString(maxPositionVariance));
    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types:
   *   * Movement
   *   * PositiveMovement
   *   * Oscillation
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("OscillationAnnotator::typeSystemInit()");

    // Movement ***********************************************
    Movement = typeSystem.getType("Movement");
    if (!Movement.isValid()) {
      log->logError("Error getting Type object for Movement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    movJnFtr = Movement.getFeatureByBaseName("jointName");

    // PositiveMovement ***********************************************
    PositiveMovement = typeSystem.getType("PositiveMovement");
    if (!PositiveMovement.isValid()) {
      log->logError("Error getting Type object for PositiveMovement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    pSpFtr = PositiveMovement.getFeatureByBaseName("startPosition");
    pEpFtr = PositiveMovement.getFeatureByBaseName("endPosition");

    // Oscillation ***********************************************
    Oscillation = typeSystem.getType("Oscillation");
    if (!Oscillation.isValid()) {
      log->logError("Error getting Type object for Oscillation");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    oscJnFtr = Oscillation.getFeatureByBaseName("jointName");

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("OscillationAnnotator::destroy()");
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
    log->logMessage("OscillationAnnotator::process() begins");

    // Intialize the cas index and the movement index iterator.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex pmIndex = cas.getAnnotationIndex(PositiveMovement);
    uima::ANIterator moveIter = cas.getAnnotationIndex(Movement).iterator();

    // Initialize reused variables.
    uima::AnnotationFS move, pMove, osc;
    std::vector<uima::AnnotationFS> posMoves;
    std::vector<double> posDiffs, timeDiffs;
    double posVar, timeVar, diff;

    // Loop through movement annotations.
    while (moveIter.isValid()) {
      move = moveIter.get();
      posMoves = utils::selectCovered(pmIndex, move);
      posDiffs.clear();
      timeDiffs.clear();

      // For every positive move which is covered by this move..
      for (std::size_t i = 0, size = posMoves.size(); i < size; i++) {
        pMove = posMoves[i];

        if (move.getStringValue(movJnFtr) == pMove.getStringValue(movJnFtr)) {
          diff = pMove.getBeginPosition() - pMove.getEndPosition();
          timeDiffs.push_back(diff);
          diff = pMove.getDoubleValue(pSpFtr) - pMove.getDoubleValue(pEpFtr);
          posDiffs.push_back(diff);
        }
      }

      posVar = utils::calculateVariance(posDiffs);
      timeVar = utils::calculateVariance(timeDiffs);

      if (
        posMoves.size() > 3 &&
        posVar < maxPositionVariance &&
        timeVar < maxTimeVariance
      ) {
        osc = cas.createAnnotation(
          Oscillation, move.getBeginPosition(), move.getEndPosition());
        osc.setStringValue(oscJnFtr, move.getStringValue(movJnFtr));
        // TODO: Osciallation frequency.
        index.addFS(osc);
      }

      moveIter.moveToNext();
    }

    log->logMessage("OscillationAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(OscillationAnnotator);
