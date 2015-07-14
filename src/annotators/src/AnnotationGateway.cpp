//////
// rta > annotators > AnnotationGateway.cpp

#include <string>
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "AnnotationGateway.hpp"
#include "AnnotationIterator.hpp"
#include "StdCoutLogger.hpp"
#include "utils.hpp"


/**
 * Constructor. Initialize Analysis engine.
 */
AnnotationGateway::AnnotationGateway(void) {
  initAE();
}


/**
 * Destructor. Clean up uima pointers.
 */
AnnotationGateway::~AnnotationGateway(void) {
  errorId = engine->destroy();
  utils::checkError(errorId, *engine);

  delete cas;
  delete engine;
  delete stdCoutLogger;
  delete fileLogger;
//  delete typeSystem;
//  delete annotatorContext;
//  delete aeDescription;
}


/**
 * Initialize the analysis engine and private class variables.
 */
void AnnotationGateway::initAE() {
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
    true, "descriptors/pipelines/Pipeline.xml", errorInfo);
  utils::checkError(errorInfo, *engine);

  // Get a new CAS.
  cas = engine->newCAS();

  // Initialize SOFA.
  // We do not use a sofa document or datastream. Instead the first Annotator
  // in the aggregate flow populates CAS with data which is then analyzed by
  // handled as SOFA with the annotators further down the stream annotating it.
  icu::UnicodeString us = "";
  cas->setDocumentText(us);

  typeSystem = &cas->getTypeSystem();
//  annotatorContext = &engine->getAnnotatorContext();
//  aeDescription = &annotatorContext->getTaeSpecifier();
}


void AnnotationGateway::run() {
  // Process the CAS.
  errorId = engine->process(*cas);
  utils::checkError(errorId, *engine);

  // Call collectionProcessComplete.
  errorId = engine->collectionProcessComplete();
}


/**
 * Retrieve a UIMA type from the type system by its name given as a unicode
 * string.
 *
 * @param  typeName Unicode string specifing the type.
 * @return UIMA type specified by the given string.
 */
uima::Type AnnotationGateway::getType(
  const icu::UnicodeString& typeName
) {
  uima::Type type = typeSystem->getType(typeName);
  if (!type.isValid()) {
    utils::checkError(UIMA_ERR_RESMGR_INVALID_RESOURCE, *engine);
  }
  return type;
}


/**
 * Get an AnnotationIterator over feature structures of a specific
 * type from the annotation index.
 *
 * @param  typeName Over which feature structures to iterate.
 * @return An custom AnnotationIterator from the uimacppext package.
 */
AnnotationIterator AnnotationGateway::getAnnotationIterator(
  const icu::UnicodeString& typeName
) {
  uima::Type type = getType(typeName);
  uima::ANIterator iterator = cas->getAnnotationIndex(type).iterator();
  return AnnotationIterator(iterator, type);
}
