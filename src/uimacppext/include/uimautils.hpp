//////
// rta > uimacppext > uimautils.hpp

#ifndef __UIMAUTILS_INCLUDE__
#define __UIMAUTILS_INCLUDE__


#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "unicode/unistr.h"  // UnicodeString
#include "uima/api.hpp"
#include "utils.hpp"


/**
 * General UIMA utility functions.
 */
namespace utils {


std::vector<double> toVector(const uima::DoubleArrayFS& fs);
std::vector<std::string> toVector(const uima::StringArrayFS& fs);
uima::StringArrayFS toStringArrayFS(
  uima::CAS& cas,
  const std::vector<std::string>& vec
);
uima::DoubleArrayFS toDoubleArrayFS(
  uima::CAS& cas,
  const std::vector<double>& vec
);
std::vector<uima::AnnotationFS> selectCovered(
  const uima::ANIndex& index,
  const uima::AnnotationFS& fs
);
std::vector<uima::AnnotationFS> selectCovered(
  uima::ANIterator& iterator,
  const uima::AnnotationFS& fs
);
void checkError(
  const uima::TyErrorId& errorId,
  const uima::AnalysisEngine& engine
);
void checkError(
  const uima::ErrorInfo& errInfo,
  const uima::AnalysisEngine& engine
);


}  // namespace utils


#endif
