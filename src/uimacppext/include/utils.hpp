//////
// rta > uimacppext > utils.hpp

#ifndef __UTILS_INCLUDE__
#define __UTILS_INCLUDE__


#include <string>
#include <vector>
#include <sstream>  // ostringstream, istringstream
#include <cstdlib>  // size_t
#include <cmath>  // pow
#include "unicode/unistr.h"  // UnicodeString
#include "uima/api.hpp"

/**
 * General C++ and UIMA utility functions.
 */
namespace utils {


// General C++ utility functions.
int toInt(const std::string& str);
std::vector<int> toInt(const std::vector<std::string>& vec);
std::string toString(double x);
std::string toString(float x);
std::string toString(int x);
std::string toString(std::size_t x);
icu::UnicodeString toUS(const std::string& str);
std::vector<std::string> split(const std::string& src, char del);
int indexOf(const std::vector<std::string>& vec, const std::string& val);
double calculateMean(const std::vector<double>& vec);
double calculateVariance(const std::vector<double>& vec);


// UIMACPP specific utility functions.
std::vector<double> arrFStoVec(const uima::DoubleArrayFS& fs);
std::vector<std::string> arrFStoVec(const uima::StringArrayFS& fs);
std::vector<uima::AnnotationFS> selectCovered(
  const uima::ANIndex& index,
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
