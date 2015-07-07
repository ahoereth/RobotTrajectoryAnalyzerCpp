/**
 * src/app.cpp
 */

#include <string>
#include "uima/api.hpp"
#include "StdCoutLogger.hpp"
#include "utils.hpp"


/**
 * Main application for initializing the UIMA aggregation flow and
 * visualizing results.
 *
 * @param  argc
 * @param  argv
 * @return
 */
int main(int argc, char* argv[]) {
  uima::TyErrorId errorId;
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
  utils::checkError(errorInfo, *engine);

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
  errorId = engine->process(*cas);
  utils::checkError(errorId, *engine);

  // Call collectionProcessComplete.
  errorId = engine->collectionProcessComplete();

  // Free resorces.
  errorId = engine->destroy();
  utils::checkError(errorId, *engine);
  delete cas;
  delete engine;
  delete stdCoutLogger;
  delete fileLogger;

  std::cout << "App: processing finished sucessfully! " << std::endl;
  return 0;
}
