//////
// rta > annotators > AnnotationGateway.cpp

#include <string>
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "AnnotationGateway.hpp"
#include "StdCoutLogger.hpp"
#include "uimautils.hpp"


/**
 * Constructor. Initialize Analysis engine.
 */
AnnotationGateway::AnnotationGateway(void) :
  pipeline("descriptors/pipelines/Pipeline.xml")
{
  // Create a resource manager instance (singleton)
  uima::ResourceManager& resourceManager =
    uima::ResourceManager::createInstance("RobotTrajectoryAnalyzer");

  // Register terminal logger.
  stdCoutLogger = new StdCoutLogger(false);
  resourceManager.registerLogger(stdCoutLogger);

  // Register file logger.
  fileLogger = new uima::FileLogger("uima.log");
  resourceManager.registerLogger(fileLogger);

  // Analysis Engine Description
  uima::XMLParser parser;
  aeDescription = new uima::AnalysisEngineDescription();
  parser.parseAnalysisEngineDescription(*aeDescription, pipeline);
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
  delete aeDescription;
}


void AnnotationGateway::run() {
  // Commit Analysis Engine Description.
  aeDescription->validate();
  aeDescription->commit();

  // Initialize Analysis Engine.
  engine = uima::Framework::createAnalysisEngine(*aeDescription, errorInfo);
  utils::checkError(errorInfo, *engine);

  // Get a new CAS.
  cas = engine->newCAS();

  // Initialize SOFA.
  // We do not use a sofa document or datastream. Instead the first Annotator
  // in the aggregate flow populates the CAS with data which is then analyzed
  // the annotators further down the stream.
  icu::UnicodeString us = "";
  cas->setDocumentText(us);

  typeSystem = &cas->getTypeSystem();

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


uima::Feature AnnotationGateway::getFeature(
  const icu::UnicodeString& typeName,
  const icu::UnicodeString& featureName
) {
  return getType(typeName).getFeatureByBaseName(featureName);
}


/**
 * Get an AnnotationIterator over feature structures of a specific
 * type from the annotation index.
 *
 * @param  typeName Over which feature structures to iterate.
 * @return An custom AnnotationIterator from the uimacppext package.
 */
uima::ANIterator AnnotationGateway::getANIterator(
  const icu::UnicodeString& typeName
) {
  return cas->getAnnotationIndex(getType(typeName)).iterator();
}


/**
 * Set an analysis engine description configuration parameter key value pair.
 *
 * @param name
 * @param value
 */
bool AnnotationGateway::setParameter(
  const icu::UnicodeString& name,
  const icu::UnicodeString& value
) {
  uima::NameValuePair* pair = aeDescription->getNameValuePair(name);
  if (pair != NULL) {
    if (pair->isModifiable()) {
      pair->setValue(value);
    } else {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }
  } else {
    uima::NameValuePair pair;
    pair.setName(name);
    pair.setValue(value);
    return (UIMA_ERR_NONE == aeDescription->setNameValuePair(pair));
  }

  return UIMA_ERR_NONE;
}
