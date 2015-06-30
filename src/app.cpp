/*
 * Copyright 2015 Alexander Hoereth
 */

#include <string>
#include "uima/api.hpp"
#include "utils.hpp"

using namespace std;
using namespace uima;


int main(int argc, char * argv[]) {
  ErrorInfo errorInfo;

  // Create a resource manager instance (singleton)
  ResourceManager::createInstance("UIMACPP_EXAMPLE_APPLICATION");

  // Initialize Analysis Engine.
  AnalysisEngine *pEngine =
    Framework::createAnalysisEngine("descriptors/Pipeline.xml", errorInfo);
  utils::checkError(errorInfo);

  // Get a new CAS.
  CAS *tcas = pEngine->newCAS();

  // We do not use a sofa document or datastream. Instead the first Annotator
  // in the aggregate flow is a Populator and fills the CAS with data which
  // is then analyzed by the annotators further down the stream.
  // Ideally this would take a more UIMA native approach using sofas.
  icu::UnicodeString us = "Some Text";
  tcas->setDocumentText(us);

  // Process the CAS.
  TyErrorId utErrorId = pEngine->process(*tcas);
  utils::checkError(utErrorId, *pEngine);

  // Call collectionProcessComplete.
  utErrorId = pEngine->collectionProcessComplete();

  // Free ressorces.
  utErrorId = pEngine->destroy();
  utils::checkError(utErrorId, *pEngine);
  delete tcas;
  delete pEngine;

  cout << "App: processing finished sucessfully! " << endl;
  return(0);
}
