/**
 * Copyright 2015 Alexander Hoereth
 */

#include <vector>
#include <string>
#include <cstdlib>  // size_t
#include "uima/api.hpp"
#include "mongo/bson/bson.h"

namespace utils {


icu::UnicodeString sToUs(const std::string& str);


uima::StringArrayFS fieldToStringArrayFS(
  const mongo::BSONElement& field,
  uima::CAS &cas
);


uima::DoubleArrayFS fieldToDoubleArrayFS(
  const mongo::BSONElement& field,
  uima::CAS &cas
);


uima::AnalysisEngineDescription * aedFromXml(
  const icu::UnicodeString& file,
  uima::ErrorInfo& errorInfo
);


void checkError(const uima::ErrorInfo& errInfo);


void checkError(
  uima::TyErrorId utErrorId,
  const uima::AnalysisEngine& crEngine
);


}  // namespace utils
