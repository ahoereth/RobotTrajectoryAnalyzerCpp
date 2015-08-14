//////
// rta > annotators > AnnotationGateway.hpp

#ifndef __ANNOTATIONGATEWAY_INCLUDE__
#define __ANNOTATIONGATEWAY_INCLUDE__


#include <string>
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "StdCoutLogger.hpp"
#include "uimautils.hpp"


/**
 * The AnnotationGateway class provides an entrance into the UIMA annotations.
 * It initializes the pipeline and annotation process and provides
 * functionality for retrieving the results.
 */
class AnnotationGateway {
 private:
  uima::AnalysisEngine* engine;
  uima::CAS* cas;
  const uima::TypeSystem* typeSystem;
  uima::AnalysisEngineDescription* aeDescription;
  StdCoutLogger* stdCoutLogger;
  uima::FileLogger* fileLogger;

  uima::TyErrorId errorId;
  uima::ErrorInfo errorInfo;

  icu::UnicodeString pipeline;

 public:
  AnnotationGateway(void);
  ~AnnotationGateway(void);
  void run();
  uima::Type getType(const icu::UnicodeString& typeName);
  uima::Feature getFeature(const icu::UnicodeString& typeName,
                           const icu::UnicodeString& featureName);
  uima::ANIterator getANIterator(const icu::UnicodeString& typeName);
  bool setParameter(const icu::UnicodeString& name,
                    const icu::UnicodeString& value);
};


#endif  // ifndef __ANNOTATIONGATEWAY_INCLUDE__
