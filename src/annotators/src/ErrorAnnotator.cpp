//////
// rta > annotators > ErrorAnnotator.cpp

#include <map>
#include <utility>  // pair
#include <cstdlib>  // size_t, abs
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


using uima::Annotator;  // required for MAKE_AE


class ErrorAnnotator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type JointTrajectoryPoint;
  uima::Type ControllerInput;
  uima::Type ControllerError;

  uima::Feature jtpPosFtr;  // JointTrajectoryPoint positions
  uima::Feature ciTypeFtr;  // ControllerInput controllerType
  uima::Feature ciJnsFtr;   // ControllerInput jointNames
  uima::Feature ciErrFtr;   // ControllerInput error
  uima::Feature ceNameFtr;  // ControllerError jointName

  // Configuration Parameters
  float minError;
  int minLength;


 public:
  /** Constructor */
  ErrorAnnotator(void) {}


  /** Destructor */
  ~ErrorAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("ErrorAnnotator::initialize()");

    // MinError ***********************************************
    minError = 0.01;
    if (annotatorContext.isParameterDefined("MinError")) {
      annotatorContext.extractValue("MinError", minError);
    }

    // MinLength **********************************************
    minLength = 5;
    if (annotatorContext.isParameterDefined("MinLength")) {
      annotatorContext.extractValue("MinLength", minLength);
    }

    log->logMessage("MinError: " + utils::toString(minError) + ", "
                    "MinLength: " + utils::toString(minLength));

    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types
   *   * JointTrajectoryPoint
   *   * ControllerInput
   *   * ControllerError
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("ErrorAnnotator::typeSystemInit()");

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jtpPosFtr = JointTrajectoryPoint.getFeatureByBaseName("positions");

    // ControllerInput ****************************************
    ControllerInput = typeSystem.getType("ControllerInput");
    if (!ControllerInput.isValid()) {
      log->logError("Error getting Type object for ControllerInput");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    ciTypeFtr = ControllerInput.getFeatureByBaseName("controllerType");
    ciJnsFtr  = ControllerInput.getFeatureByBaseName("jointNames");
    ciErrFtr  = ControllerInput.getFeatureByBaseName("error");

    // ControllerError ****************************************
    ControllerError = typeSystem.getType("ControllerError");
    if (!ControllerError.isValid()) {
      log->logError("Error getting Type object for ControllerError");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    ceNameFtr = ControllerError.getFeatureByBaseName("jointName");

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("ErrorAnnotator::destroy()");
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
    log->logMessage("ErrorAnnotator::process() begins");

    // Initialize required indices.
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex ciIndex = cas.getAnnotationIndex(ControllerInput);
    uima::ANIterator ciIter = ciIndex.iterator();

    // Mapping from **controller** to **joint name** to **begin/end-pair**
    typedef std::map<uima::UnicodeStringRef, std::pair<int, int> > jointsType;
    std::map<uima::UnicodeStringRef, jointsType> ctrls;
    std::pair<int, int> error;
    jointsType joints;
    uima::AnnotationFS ci, ce;
    uima::DoubleArrayFS posError;
    uima::StringArrayFS names;
    bool isError;

    // Iterate over all Controler Input annotations.
    while (ciIter.isValid()) {
      ci = ciIter.get();
      joints = ctrls[ci.getStringValue(ciTypeFtr)];
      names = ci.getStringArrayFSValue(ciJnsFtr);
      posError = ci.getFSValue(ciErrFtr).getDoubleArrayFSValue(jtpPosFtr);

      // Iterate over all error positions.
      for (size_t i = 0, size = posError.size(); i < size; i++) {
        error = joints[names.get(i)];
        isError = (std::abs(posError.get(i)) >= minError);

        // Is the error already ongoing?
        if (0 != error.first) {
          // Error continues.
          if (isError) {
            error.second = ci.getBeginPosition();
          }

          // Error just ended OR this is the last controller input iteration.
          if (!isError || !ciIter.peekNext().isValid()) {
            // Is the error significant enough to annotate?
            if ((error.second - error.first) >= minLength) {
              ce = cas.createAnnotation(ControllerError,
                                        error.first, error.second);
              ce.setStringValue(ceNameFtr, names.get(i));
              index.addFS(ce);
            }
            error.first = error.second = 0;  // Reset error annotation.
          }

        // Is there a new error to detect?
      } else if (isError) {
          error.first = error.second = ci.getBeginPosition();
        }
      }

      ciIter.moveToNext();
    }

    log->logMessage("ErrorAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(ErrorAnnotator);
