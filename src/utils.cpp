/**
 * src/utils.cpp
 */

#include <string>
#include <vector>
#include <sstream>  // ostringstream
#include <cstdlib>  // size_t
#include <cmath>  // pow
#include "unicode/unistr.h"  // UnicodeString
#include "uima/api.hpp"
#include "utils.hpp"


namespace utils {


/**
 * Get a vector of annotations from the given annotation index constrained by a
 * 'covering' annotation. Iterates over all annotations in the given index to
 * find the covered annotations. Covered means contained by the given
 * annoation's begin and end positions.
 *
 * @see uimaj's org.apache.uima.fit.util.JCasUtil
 * @param  index Annotation index containing the annotations to iterate over.
 * @param  fs    Annotation FS for constraining the annotations from `index`.
 * @return Vector containing all annotations from `index` covered by the given
 *         annotation feature structure.
 */
std::vector<uima::AnnotationFS> selectCovered(
  const uima::ANIndex& index,
  const uima::AnnotationFS& fs
) {
  std::vector<uima::AnnotationFS> result;
  uima::AnnotationFS item;
  std::size_t begin = fs.getBeginPosition();
  std::size_t end = fs.getEndPosition();

  uima::ANIterator iter = index.iterator();
  while (iter.isValid()) {
    item = iter.get();
    iter.moveToNext();

    if (item.getBeginPosition() >= begin && item.getEndPosition() <= end) {
      result.push_back(item);
    }
  }

  return result;
}


/**
 * Convert a array feature structure to a vector.
 *
 * @param  fs Array feature structure to convert.
 * @return Resulting vector.
 */
std::vector<double> arrFStoVec(
  const uima::DoubleArrayFS& fs
) {
  std::size_t size = fs.size();
  std::vector<double> result(size, 0);

  for (std::size_t i = 0; i < size; i++) {
    result[i] = fs.get(i);
  }

  return result;
}


/**
 * Calculate the variance in the values of a vector.
 *
 * @param  vec Vector of double values.
 * @return Variance as double.
 */
double calculateVariance(
  const std::vector<double>& vec
) {
  double variance = 0, mean = 0;

  // MEAN
  for (size_t i = 0, size = vec.size(); i < size; i++) {
    mean += vec[i] / size;
  }

  // VARIANCE
  for (size_t i = 0, size = vec.size(); i < size; i++) {
    variance += pow(mean - vec[i], 2.0) / size;
  }

  return variance;
}


/**
 * Convert a C++ standard string to a ICU Unicode String as required by many
 * UIMA applications.
 *
 * @param  str Std string to convert to a unicode string.
 * @return The resulting unicode string.
 */
icu::UnicodeString sToUs(const std::string& str) {
  return icu::UnicodeString(str.data(), str.length(), US_INV);
}


/**
 * Basic type conversion.
 *
 * @param  x Double to convert to string.
 * @return The resulting string.
 */
std::string toString(double x) {
  std::ostringstream ss;
  ss << x;
  return ss.str();
}


/**
 * Basic type conversion.
 *
 * @param  x Float to convert to string.
 * @return The resulting string.
 */
std::string toString(float x) {
  return toString(static_cast<double>(x));
}


/**
 * Basic type conversion.
 *
 * @param  x Integer to convert to string.
 * @return The resulting string.
 */
std::string toString(int x) {
  return toString(static_cast<double>(x));
}


/**
 * Basic type conversion.
 *
 * @param  x Unsigned integer to convert to string.
 * @return The resulting string.
 */
std::string toString(std::size_t x) {
  return toString(static_cast<double>(x));
}


/**
 * Helper routine to check and report errors. Exits the program on error.
 *
 * @param errorId An error id which can be used to retrieve more information.
 * @param engine  The analysis engine in which context the error occured.
 */
void checkError(
  const uima::TyErrorId& errorId,
  const uima::AnalysisEngine& engine
) {
  if (errorId != UIMA_ERR_NONE) {
    uima::LogFacility& log = engine.getAnnotatorContext().getLogger();
    log.logError(uima::AnalysisEngine::getErrorIdAsCString(errorId));
    exit(static_cast<int>(errorId));
  }
}


/**
 * Helper routine to check and report errors. Exits the program on error.
 *
 * Delegates to the preceding checkError method.
 *
 * @param errInfo ErrorInfo object containing a complete error information set.
 * @param engine  The analysis engine in which context the error occured.
 */
void checkError(
  const uima::ErrorInfo& errInfo,
  const uima::AnalysisEngine& engine
) {
  checkError(errInfo.getErrorId(), engine);
}


}  // namespace utils
