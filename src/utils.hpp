/**
 * src/utils.hpp
 */

#include <string>
#include <vector>
#include <sstream>  // ostringstream
#include <cstdlib>  // size_t
#include <cmath>  // pow
#include "unicode/unistr.h"  // UnicodeString
#include "uima/api.hpp"

/**
 * General C++ and UIMA helper functions. Some of those are reimplementations
 * of functions which are part of the C++11 standard (which is not supported
 * by uimacpp).
 */
namespace utils {

std::vector<uima::AnnotationFS> selectCovered(
  const uima::ANIndex& index,
  const uima::AnnotationFS& fs
);
std::vector<double> arrFStoVec(const uima::DoubleArrayFS& fs);
double calculateVariance(const std::vector<double>& vec);
icu::UnicodeString sToUs(const std::string& str);
std::string toString(double x);
std::string toString(float x);
std::string toString(int x);
std::string toString(std::size_t x);
void checkError(
  const uima::TyErrorId& errorId,
  const uima::AnalysisEngine& engine
);
void checkError(
  const uima::ErrorInfo& errInfo,
  const uima::AnalysisEngine& engine
);

}  // namespace utils
