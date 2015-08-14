//////
// rta > uimacppext > uimautils.cpp


#include <string>
#include <vector>
#include <cstdlib>  // size_t
#include "unicode/unistr.h"  // UnicodeString
#include "uima/api.hpp"
#include "utils.hpp"
#include "uimautils.hpp"


namespace utils {


/**
 * Type conversion
 *
 * @param  fs Double array feature structure.
 * @return Vector of doubles.
 */
std::vector<double> toVector(
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
 * Type conversion.
 *
 * @param  fs String array feature structure.
 * @return Vector of standard strings.
 */
std::vector<std::string> toVector(
  const uima::StringArrayFS& fs
) {
  std::size_t size = fs.size();
  std::vector<std::string> result(size, "");

  for (std::size_t i = 0; i < size; i++) {
    fs.get(i).extractUTF8(result[i]);
  }

  return result;
}


uima::StringArrayFS toStringArrayFS(
  uima::CAS& cas,
  const std::vector<std::string>& vec
) {
  uima::StringArrayFS fs = cas.createStringArrayFS(vec.size());
  for (std::size_t i = 0; i < vec.size(); ++i) {
    fs.set(i, toUS(vec[i]));
  }
  return fs;
}


uima::DoubleArrayFS toDoubleArrayFS(
  uima::CAS& cas,
  const std::vector<double>& vec
) {
  uima::DoubleArrayFS fs = cas.createDoubleArrayFS(vec.size());
  for (std::size_t i = 0; i < vec.size(); ++i) {
    fs.set(i, vec[i]);
  }
  return fs;
}


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
  uima::ANIterator iter = index.iterator();
  return selectCovered(iter, fs);
}

std::vector<uima::AnnotationFS> selectCovered(
  uima::ANIterator& iter,
  const uima::AnnotationFS& fs
) {
  std::vector<uima::AnnotationFS> result;
  uima::AnnotationFS item;
  std::size_t begin = fs.getBeginPosition();
  std::size_t end = fs.getEndPosition();

  uima::FeatureStructure original = iter.get();

  iter.moveToFirst();
  while (iter.isValid()) {
    item = iter.get();
    iter.moveToNext();

    if (item.getBeginPosition() >= begin && item.getEndPosition() <= end) {
      result.push_back(item);
    }
  }

  iter.moveTo(original);
  return result;
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
