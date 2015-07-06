/**
 * src/MovementAnnotator.cpp
 */

#include <string>
#include <vector>
#include <list>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include "uima/api.hpp"
#include "unicode/unistr.h"
#include "utils.hpp"


using std::string;
using std::vector;
using uima::Annotator;  // required for MAKE_AE
using uima::Feature;


class MovementAnnotator : public Annotator {
 private:
  uima::LogFacility* log;
  uima::CAS* currentCas;
  uima::Type JointState;
  uima::Type JointTrajectoryPoint;
  uima::Type Movement;

  float minVariance;
  std::size_t observedJointStates;


 public:
  /** Constructor */
  MovementAnnotator(void) {}


  /** Destructor */
  ~MovementAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("MovementAnnotator: initialize()");

    minVariance = 0.000003f;
    if (annotatorContext.isParameterDefined("MinVariance")) {
      annotatorContext.extractValue("MinVariance", minVariance);
    }

    observedJointStates = 10;
    if (annotatorContext.isParameterDefined("ObservedJointStates")) {
      annotatorContext.extractValue("ObservedJointStates", observedJointStates);
    }

    log->logMessage("MinVariance: " + utils::toString(minVariance) + ", "
      "ObservedJointStates: " + utils::toString(observedJointStates));

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
    log->logMessage("MovementAnnotator:: typeSystemInit() begins");

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

    // Movement ***********************************************
    Movement = typeSystem.getType("Movement");
    if (!Movement.isValid()) {
      log->logError("Error getting Type object for Movement");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    log->logMessage("MovementAnnotator:: typeSystemInit() ends");
    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("MovementAnnotator: destroy()");
    return UIMA_ERR_NONE;
  }


  /**
   * Calculate the variances in a column of a list of double array feature
   * structure rows.
   *
   * @param  posFS List of double array feature structures.
   * @return Vector of the same size as any of the passed double array
   *         feature structures.
   */
  std::vector<double> calculateVariances(
    std::list<uima::DoubleArrayFS> posFS
  ) {
    std::vector<double> means(observedJointStates, 0);
    std::vector<double> variances(observedJointStates, 0);

    // MEANS
    for (
      std::list<uima::DoubleArrayFS>::const_iterator positions = posFS.begin();
      positions != posFS.end(); ++positions
    ) {
      for (std::size_t i = 0; i < observedJointStates; i++) {
        means[i] += positions->get(i) / observedJointStates;
      }
    }

    // VARIANCES
    for (
      std::list<uima::DoubleArrayFS>::const_iterator positions = posFS.begin();
      positions != posFS.end(); ++positions
    ) {
      for (std::size_t i = 0; i < observedJointStates; i++) {
        variances[i] +=
          (pow(means[i] - positions->get(i), 2.0) / observedJointStates);
      }
    }

    return variances;
  }


  /**
   * Create a movement annotation feature structure given a name, a begin and an
   * end position.
   *
   * @param  name Unicode string giving the feature structure's name feature
   *              value.
   * @param  from Unsigned integer giving the annotation begin position.
   * @param  to   Unsigned integer giving the annotation end position.
   * @return      Movement annotation feature structure.
   */
  uima::AnnotationFS moveAnnotation(
    icu::UnicodeString name,
    std::size_t from,
    std::size_t to
  ) {
    uima::AnnotationFS move = currentCas->createAnnotation(Movement, from, to);
    move.setStringValue(Movement.getFeatureByBaseName("jointName"), name);
    return move;
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
    log->logMessage("MovementAnnotator::process() begins");

    // Save cas for use in other member functions.
    currentCas = &cas;

    // Initialize features.
    Feature seqFtr = JointState.getFeatureByBaseName("seq");
    Feature jtpFtr = JointState.getFeatureByBaseName("jointTrajectoryPoint");
    Feature nameFtr = JointState.getFeatureByBaseName("name");
    Feature posFtr = JointTrajectoryPoint.getFeatureByBaseName("positions");
    Feature jnFtr = Movement.getFeatureByBaseName("jointName");

    // Intialize indices and the index iterator.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIterator jsIter = cas.getAnnotationIndex(JointState).iterator();

    // Initialize reused variables.
    uima::FeatureStructure js, jtp;
    uima::StringArrayFS names;
    uima::AnnotationFS move;
    uima::ArrayFS moves = cas.createArrayFS(observedJointStates);
    std::list<uima::DoubleArrayFS> prevPositions;
    std::vector<bool> movingStates(observedJointStates, false);
    std::vector<double> variances;
    std::size_t seq;

    // Init log stream.
    uima::LogStream& logstream = log->getLogStream(uima::LogStream::EnMessage);

    // Loop through joint states.
    while (jsIter.isValid()) {
      js = jsIter.get();
      jtp = js.getFSValue(jtpFtr);
      seq = js.getIntValue(seqFtr);
      names = js.getStringArrayFSValue(nameFtr);
      prevPositions.push_back(jtp.getDoubleArrayFSValue(posFtr));

      // If the list is too long pop the first element.
      if (prevPositions.size() > observedJointStates) {
        prevPositions.pop_front();
      }

      // If the list is too short continue adding the next element.
      if (prevPositions.size() < observedJointStates) {
        continue;
      }

      // Calculate variances in every joint's positions to detect movements.
      variances = calculateVariances(prevPositions);

      // Iterate over the observed variances.
      for (std::size_t i = 0; i < variances.size(); i++) {
        move = static_cast<uima::AnnotationFS>(moves.get(i));
        icu::UnicodeString name = names.get(i).getBuffer();

        if (1 == move.getBeginPosition()) {  // Initial iteration.
          moves.set(i, moveAnnotation(name, seq, seq));
        } else {  // Consecutive iterations.
          bool moving = (variances[i] >= minVariance);

          if (                                 // Movement state:
            (!movingStates[i] &&  moving) ||   // wasn't + is
             (movingStates[i] && !moving)      // was    + isn't
          ) {
            logstream << name << " :: movement detected" << std::endl;
            index.addFS(move);
            moves.set(i, moveAnnotation(name, seq, seq));

          } else if (                         // Movement state:
            (!movingStates[i] && !moving) ||  // wasn't + isn't
             (movingStates[i] &&  moving)     // was    + is
          ) {
            moves.set(i, moveAnnotation(name, move.getBeginPosition(), seq));
          }
        }
      }

      jsIter.moveToNext();
    }

    // Flush log.
    logstream.flush();

    // Add all remaining movements to the index - it might just be that the
    // joints didn't move at all and those are still their initial states.
    for (std::size_t i = 0; i < moves.size(); i++) {
      move = static_cast<uima::AnnotationFS>(moves.get(i));
      index.addFS(move);
    }

    log->logMessage("MovementAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(MovementAnnotator);
