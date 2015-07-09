//////
// src/AnnotationGateway.hpp

#ifndef __ANNOTATIONGATEWAY_INCLUDE__
#define __ANNOTATIONGATEWAY_INCLUDE__


#include "uima/api.hpp"
#include "AnnotationGateway.hpp"
#include "StdCoutLogger.hpp"
#include "utils.hpp"


/**
 * The AnnotationGateway class provides an entrance into the UIMA annotations.
 * It initializes the pipeline and annotation process and provides
 * functionality for retrieving the results.
 */
class AnnotationGateway {
 private:
  uima::AnalysisEngine* engine;
  uima::CAS* cas;
  StdCoutLogger* stdCoutLogger;
  uima::FileLogger* fileLogger;


 public:
  AnnotationGateway(void);
  ~AnnotationGateway(void);
  void run();
};


#endif  // ifndef __ANNOTATIONGATEWAY_INCLUDE__
