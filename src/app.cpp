/*
 * src/app.cpp
 */

#include <string>
#include "uima/api.hpp"
#include "StdCoutLogger.hpp"


/**
 * Helper routine to check and report errors. Exits the program on error.
 *
 * @param {TyErrorId}      errorId
 * @param {AnalysisEngine} crEngine
 */
void checkError(
  const uima::TyErrorId& errorId,
  const uima::AnalysisEngine& engine
) {
  if (errorId != UIMA_ERR_NONE) {
    uima::LogFacility& log = engine.getAnnotatorContext().getLogger();
    log.logError(uima::AnalysisEngine::getErrorIdAsCString(errorId));
    exit(static_cast<int>(errorId));
  }
}


/**
 * Helper routine to check and report errors. Exits the program on error.
 *
 * @param {ErrorInfo} errInfo
 */
void checkError(
  const uima::ErrorInfo& errInfo,
  const uima::AnalysisEngine& engine
) {
  checkError(errInfo.getErrorId(), engine);
}


/**
 * Main application for initializing the UIMA aggregation flow and
 * visualizing results.
 *
 * @param  {int}    argc
 * @param  {char[]} argv
 * @return {int}
 */
int main(int argc, char* argv[]) {
  uima::ErrorInfo errorInfo;

  // Create a resource manager instance (singleton)
  uima::ResourceManager& resourceManager =
    uima::ResourceManager::createInstance("RobotTrajectoryAnalyzer");

  // Register terminal logger.
  StdCoutLogger* stdCoutLogger = new StdCoutLogger(false);
  resourceManager.registerLogger(stdCoutLogger);

  // Register file logger.
  uima::FileLogger* fileLogger = new uima::FileLogger("uima.log");
  resourceManager.registerLogger(fileLogger);

  // Initialize Analysis Engine.
  uima::AnalysisEngine* engine = uima::Framework::createAnalysisEngine(
    true, "descriptors/Pipeline.xml", errorInfo);
  checkError(errorInfo, *engine);

  // Get a new CAS.
  uima::CAS* cas = engine->newCAS();

  // Initialize SOFA.
  // We do not use a sofa document or datastream. Instead the first Annotator
  // in the aggregate flow is a Populator and fills the CAS with data which
  // is then analyzed by the annotators further down the stream.
  // Ideally this would take a more UIMA native approach using sofas.
  icu::UnicodeString us = "Some Text";
  cas->setDocumentText(us);

  // Process the CAS.
  uima::TyErrorId errorId = engine->process(*cas);
  checkError(errorId, *engine);

  // Call collectionProcessComplete.
  errorId = engine->collectionProcessComplete();

  // Free resorces.
  errorId = engine->destroy();
  checkError(errorId, *engine);
  delete cas;
  delete engine;
  delete stdCoutLogger;
  delete fileLogger;

  std::cout << "App: processing finished sucessfully! " << std::endl;
  return(0);
}
