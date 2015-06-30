/*
 * Copyright 2015 Alexander Hoereth
 */

#include <string>
#include "uima/api.hpp"


/**
 * Helper routine to check and report errors. Exits the program on error.
 *
 * @param {TyErrorId}      utErrorId
 * @param {AnalysisEngine} crEngine
 */
void checkError(
  uima::TyErrorId utErrorId,
  const uima::AnalysisEngine& crEngine
) {
  if (utErrorId != UIMA_ERR_NONE) {
    std::cerr << std::endl
      << "   *** Error info:" << std::endl
      << "Error number        : " << utErrorId << std::endl
      << "Error string        : "
      << uima::AnalysisEngine::getErrorIdAsCString(utErrorId) << std::endl;

    const TCHAR* errStr =
      crEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr();
    if (errStr != NULL) {
      std::cerr << "  Last logged message : "  << errStr << std::endl;
    }

    exit(static_cast<int>(utErrorId));
  }
}


/**
 * Helper routine to check and report errors. Exits the program on error.
 *
 * @param {ErrorInfo}      errInfo
 */
void checkError(const uima::ErrorInfo& errInfo) {
  if (errInfo.getErrorId() != UIMA_ERR_NONE) {
    std::cerr << std::endl
      << "   *** Error info:" << std::endl
      << "Error string  : "
      << uima::AnalysisEngine::getErrorIdAsCString(errInfo.getErrorId())
      << errInfo << std::endl;

    exit(static_cast<int>(errInfo.getErrorId()));
  }
}


/**
 * Main application for initializing the UIMA aggregation flow and
 * visualizing results.
 *
 * @param  {int}    argc
 * @param  {char[]} argv
 * @return {int}
 */
int main(int argc, char * argv[]) {
  uima::ErrorInfo errorInfo;

  // Create a resource manager instance (singleton)
  uima::ResourceManager::createInstance("UIMACPP_EXAMPLE_APPLICATION");

  // Initialize Analysis Engine.
  uima::AnalysisEngine *pEngine = uima::Framework::createAnalysisEngine(
    "descriptors/Pipeline.xml", errorInfo);
  checkError(errorInfo);

  // Get a new CAS.
  uima::CAS *tcas = pEngine->newCAS();

  // We do not use a sofa document or datastream. Instead the first Annotator
  // in the aggregate flow is a Populator and fills the CAS with data which
  // is then analyzed by the annotators further down the stream.
  // Ideally this would take a more UIMA native approach using sofas.
  icu::UnicodeString us = "Some Text";
  tcas->setDocumentText(us);

  // Process the CAS.
  uima::TyErrorId utErrorId = pEngine->process(*tcas);
  checkError(utErrorId, *pEngine);

  // Call collectionProcessComplete.
  utErrorId = pEngine->collectionProcessComplete();

  // Free ressorces.
  utErrorId = pEngine->destroy();
  checkError(utErrorId, *pEngine);
  delete tcas;
  delete pEngine;

  std::cout << "App: processing finished sucessfully! " << std::endl;
  return(0);
}
