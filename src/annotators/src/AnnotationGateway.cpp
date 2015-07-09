//////
// src/AnnotationGateway.cpp

#include "uima/api.hpp"
#include "AnnotationGateway.hpp"
#include "StdCoutLogger.hpp"
#include "utils.hpp"

AnnotationGateway::AnnotationGateway(void) {}


/**
 * Destructor. Clean up uima pointers.
 */
AnnotationGateway::~AnnotationGateway(void) {
  delete cas;
  delete engine;
  delete stdCoutLogger;
  delete fileLogger;
}


void AnnotationGateway::run() {
  uima::TyErrorId errorId;
  uima::ErrorInfo errorInfo;

  // Create a resource manager instance (singleton)
  uima::ResourceManager& resourceManager =
    uima::ResourceManager::createInstance("RobotTrajectoryAnalyzer");

  // Register terminal logger.
  stdCoutLogger = new StdCoutLogger(false);
  resourceManager.registerLogger(stdCoutLogger);

  // Register file logger.
  fileLogger = new uima::FileLogger("uima.log");
  resourceManager.registerLogger(fileLogger);

  // Initialize Analysis Engine.
  engine = uima::Framework::createAnalysisEngine(
    true, "descriptors/analysis_engines/Pipeline.xml", errorInfo);
  utils::checkError(errorInfo, *engine);

  // Get a new CAS.
  cas = engine->newCAS();

  // Initialize SOFA.
  // We do not use a sofa document or datastream. Instead the first Annotator
  // in the aggregate flow is a Populator and fills the CAS with data which
  // is then analyzed by the annotators further down the stream.
  // Ideally this would take a more UIMA native approach using sofas.
  icu::UnicodeString us = "";
  cas->setDocumentText(us);

  // Process the CAS.
  errorId = engine->process(*cas);
  utils::checkError(errorId, *engine);

  // Call collectionProcessComplete.
  errorId = engine->collectionProcessComplete();

  // Free resorces.
  errorId = engine->destroy();
  utils::checkError(errorId, *engine);
}
