/**
 * Copyright 2015 Alexander Hoereth
 */

#include <vector>
#include <string>
#include <cstdlib>  // size_t
//#include <sys/stat.h>
#include "uima/api.hpp"
#include "utils.hpp"
#include "mongo/bson/bson.h"

namespace utils {

/*
std::string usToS(const icu::UnicodeString& us) {
  std::string s;
  us.extract((int32_t)0, us.length(), s);
  return s;
}*/


/**
 * Convert a C++ standard string to a ICU Unicode String as required by many
 * UIMA applications.
 *
 * @param  {std::string} str
 * @return {icu::UnicodeString}
 */
icu::UnicodeString sToUs(const std::string& str) {
  return icu::UnicodeString(str.data(), str.length(), US_INV);
}


/*
bool file_exists(const icu::UnicodeString& filename) {
  struct stat buffer;
  return (stat (usToS(filename).c_str(), &buffer) == 0);
}*/


/**
 * Convert a mongo BSON element to a UIMA CAS String Array Feature Structure.
 *
 * @param  {mongo::BSONElement}  field
 * @param  {uima::CAS}           cas
 * @return {uima::StringArrayFS}
 */
uima::StringArrayFS fieldToStringArrayFS(
  const mongo::BSONElement& field,
  uima::CAS& cas
) {
  std::vector<mongo::BSONElement> vec = field.Array();
  uima::StringArrayFS fs = cas.createStringArrayFS(vec.size());

  for (std::size_t i = 0; i < vec.size(); ++i) {
    fs.set(i, sToUs(vec[i].String()));
  }

  return fs;
}


/**
 * Convert a mongo BSON element to a UIMA CAS Double Array Feature Structure.
 *
 * @param  {mongo::BSONElement}  field
 * @param  {uima::CAS}           cas
 * @return {uima::StringDoubleFS}
 */
uima::DoubleArrayFS fieldToDoubleArrayFS(
  const mongo::BSONElement& field,
  uima::CAS& cas
) {
  std::vector<mongo::BSONElement> vec = field.Array();
  uima::DoubleArrayFS fs = cas.createDoubleArrayFS(vec.size());

  for (std::size_t i = 0; i < vec.size(); ++i) {
    fs.set(i, vec[i].Double());
  }

  return fs;
}


/**
 *
 *
 * @see createAnalysisEngine() in $UIMACPP_HOME/src/engine.cpp
 * @param  file [description]
 * @return      [description]
 */
uima::AnalysisEngineDescription * aedFromXml(
  const icu::UnicodeString& filename,
  uima::ErrorInfo& errorInfo
) {
  errorInfo.reset();
  if (!uima::ResourceManager::hasInstance()) {
    errorInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
    return NULL;
  }

  uima::AnalysisEngineDescription * aed = new uima::AnalysisEngineDescription();

  if (aed == NULL) {
    errorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
    return NULL;
  }

  // TODO(ahoereth): Ensure for file existence.

  uima::XMLParser builder;
  builder.parseAnalysisEngineDescription(*aed, filename);
  aed->validate();
  aed->commit();

  return aed;
}


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
      << "   *** ExampleApplication - Error info:" << std::endl
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
      << "   *** ExampleApplication - Error info:" << std::endl
      << "Error string  : "
      << uima::AnalysisEngine::getErrorIdAsCString(errInfo.getErrorId())
      << errInfo << std::endl;

    exit(static_cast<int>(errInfo.getErrorId()));
  }
}


}  // namespace utils
