//////
// rta > annotators > MovementDirectionAnnotator.cpp

#include <vector>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "utils.hpp"


using uima::Annotator;  // required for MAKE_AE
using uima::Feature;


class MovementDirectionAnnotator : public Annotator {
 private:
  static const int POSITIVE =  1;
  static const int NEGATIVE = -1;

  uima::LogFacility* log;
  uima::CAS* currentCas;

  uima::Type JointState;
  uima::Type JointTrajectoryPoint;
  uima::Type Movement;
  uima::Type PositiveMovement;
  uima::Type NegativeMovement;

  uima::Feature jsnFtr;  // Joint State Name Feature
  uima::Feature jtpFtr;  // Joint Trajectory Point Feature
  uima::Feature posFtr;  // Joint Trajectory Point Positions Feature
  uima::Feature pJnFtr;  // Positive Joint Name Feature
  uima::Feature pSpFtr;  // Positive Start Position Feature
  uima::Feature pEpFtr;  // Positive End Position Feature
  uima::Feature nJnFtr;  // Negative Joint Name Feature
  uima::Feature nSpFtr;  // Negative Start Position Feature
  uima::Feature nEpFtr;  // Negative End Position Feature

  float minVariance;
  std::size_t observedJointStates;


 public:
  /** Constructor */
  MovementDirectionAnnotator(void) {}


  /** Destructor */
  ~MovementDirectionAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("MovementDirectionAnnotator: initialize()");
    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types:
   *   * JointState
   *   * JointTrajectoryPoint
   *   * Movement
   *   * NegativeMovement
   *   * PositiveMovement
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("MovementDirectionAnnotator:: typeSystemInit() begins");

    // JointState *********************************************
    JointState = typeSystem.getType("JointState");
    if (!JointState.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jsnFtr = JointState.getFeatureByBaseName("name");
    jtpFtr = JointState.getFeatureByBaseName("jointTrajectoryPoint");

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    posFtr = JointTrajectoryPoint.getFeatureByBaseName("positions");

    // Movement ***********************************************
    Movement = typeSystem.getType("Movement");
    if (!Movement.isValid()) {
      log->logError("Error getting Type object for Movement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    // PositiveMovement ***************************************
    PositiveMovement = typeSystem.getType("PositiveMovement");
    if (!PositiveMovement.isValid()) {
      log->logError("Error getting Type object for PositiveMovement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    pJnFtr = PositiveMovement.getFeatureByBaseName("jointName");
    pSpFtr = PositiveMovement.getFeatureByBaseName("startPosition");
    pEpFtr = PositiveMovement.getFeatureByBaseName("endPosition");

    // NegativeMovement ***************************************
    NegativeMovement = typeSystem.getType("NegativeMovement");
    if (!NegativeMovement.isValid()) {
      log->logError("Error getting Type object for NegativeMovement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    nJnFtr = PositiveMovement.getFeatureByBaseName("jointName");
    nSpFtr = NegativeMovement.getFeatureByBaseName("startPosition");
    nEpFtr = NegativeMovement.getFeatureByBaseName("endPosition");

    log->logMessage("MovementDirectionAnnotator:: typeSystemInit() ends");
    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("MovementDirectionAnnotator: destroy()");
    return UIMA_ERR_NONE;
  }


  /**
   * Get the position of a joint from a joint state given its distinct name.
   *
   * @param  js   A joint state containing name and positions FS arrays.
   * @param  name Joint name for which position to look.
   * @return Joint position when found, otherwise -1.
   */
  double getPosByName(
    const uima::FeatureStructure& js,
    const uima::UnicodeStringRef& name
  ) {
    uima::StringArrayFS names = js.getStringArrayFSValue(jsnFtr);
    uima::DoubleArrayFS positions =
      js.getFSValue(jtpFtr).getDoubleArrayFSValue(posFtr);

    double result = -1;
    for (size_t i = 0, size = names.size(); i < size; ++i) {
      if (names.get(i) == name) {
        result = positions.get(i);
        break;
      }
    }

    return result;
  }


  /**
   * Create an annoptation of type PositiveMovement with timely and positional
   * begin and end positions.
   *
   * @param  begin    Value for the annotation begin position - time.
   * @param  end      Value for the annotation end position - time.
   * @param  posStart Value for the startPosition feature - position.
   * @param  posEnd   Value for the endPosition feature - position.
   * @return Resulting PositiveMovement annotation feature structure.
   */
  uima::AnnotationFS posMoveAnnotation(
    const uima::UnicodeStringRef& name,
    const std::size_t& begin,
    const std::size_t& end,
    const double& posStart,
    const double& posEnd
  ) {
    uima::AnnotationFS fs;
    fs = currentCas->createAnnotation(PositiveMovement, begin, end);
    fs.setStringValue(pJnFtr, name);
    fs.setDoubleValue(pSpFtr, posStart);
    fs.setDoubleValue(pEpFtr, posEnd);
    return fs;
  }


  /**
   * Create an annoptation of type NegativeMovement with timely and positional
   * begin and end positions.
   *
   * @param  begin    Value for the annotation begin position - time.
   * @param  end      Value for the annotation end position - time.
   * @param  posStart Value for the startPosition feature - position.
   * @param  posEnd   Value for the endPosition feature - position.
   * @return Resulting NegativeMovement annotation feature structure.
   */
  uima::AnnotationFS negMoveAnnotation(
    const uima::UnicodeStringRef& name,
    const std::size_t& begin,
    const std::size_t& end,
    const double& posStart,
    const double& posEnd
  ) {
    uima::AnnotationFS fs;
    fs = currentCas->createAnnotation(NegativeMovement, begin, end);
    fs.setStringValue(pJnFtr, name);
    fs.setDoubleValue(nSpFtr, posStart);
    fs.setDoubleValue(nEpFtr, posEnd);
    return fs;
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
    log->logMessage("MovementDirectionAnnotator::process() begins");

    // Save cas for use in other member functions.
    currentCas = &cas;

    // Intialize the cas index and the movement index iterator.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex jsIndex = cas.getAnnotationIndex(JointState);
    uima::ANIterator moveIter = cas.getAnnotationIndex(Movement).iterator();

    // Initialize reused variables.
    Feature jnFtr = Movement.getFeatureByBaseName("jointName");
    uima::AnnotationFS move, js;
    uima::UnicodeStringRef name;
    std::vector<uima::AnnotationFS> jointStates;
    double pos, posDiff, posPrev = 0, posStart = 0;
    std::size_t begin = 0, end;
    int direction;

    // Loop through movement annotations.
    while (moveIter.isValid()) {
      move = moveIter.get();
      jointStates = utils::selectCovered(jsIndex, move);
      direction = 0;

      // Loop through joint states related to this movement.
      for (std::size_t i = 0; i < jointStates.size(); i++) {
        js = jointStates[i];
        end = js.getEndPosition();
        begin = js.getBeginPosition();
        name = move.getStringValue(jnFtr);
        pos = getPosByName(js, name);
        posStart = pos;
        posDiff = pos - posPrev;

        // Act acording to movement direction.
        switch (direction) {
          case POSITIVE:
            if (0 > posDiff) {  // Direction changed, now negative.
              index.addFS(negMoveAnnotation(name, begin, end, posStart, pos));
              direction = NEGATIVE;
              posStart = pos;
              begin = js.getBeginPosition();
            }
            break;
          case NEGATIVE:
            if (0 < posDiff) {  // Direction changed, now positive.
              index.addFS(posMoveAnnotation(name, begin, end, posStart, pos));
              direction = POSITIVE;
              posStart = pos;
              begin = js.getBeginPosition();
            }
            break;
          default:  // Only entered in initial iteration.
            direction = (0 <= posDiff) ? POSITIVE : NEGATIVE;
        }

        posPrev = pos;
      }

      moveIter.moveToNext();
    }

    log->logMessage("MovementDirectionAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(MovementDirectionAnnotator);
