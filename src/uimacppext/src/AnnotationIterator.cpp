//////
// rta > uimacppext > AnnotationStream.cpp

#include <string>
#include <vector>
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "AnnotationIterator.hpp"
#include "utils.hpp"


AnnotationIterator::AnnotationIterator(
  const uima::ANIterator& iterator,
  const uima::Type& type
) : iterator(iterator), type(type) {}


/**
 * Retrieve a vector of double values from the feature structure currently at
 * the head of the iterator.
 *
 * @param  featureName The type name by which to retrieve the data.
 * @return The data in easy to parseable vector form.
 */
std::vector<double> AnnotationIterator::getDoubleVector(
  const icu::UnicodeString& featureName
) {
  uima::Feature feature = type.getFeatureByBaseName(featureName);
  return utils::arrFStoVec(iterator.get().getDoubleArrayFSValue(feature));
}


/**
 * Retrieve a vector of string values from the feature structure currently at
 * the head of the iterator.
 *
 * @param  featureName The type name by which to retrieve the data.
 * @return The data in easy to parseable vector form.
 */
std::vector<std::string> AnnotationIterator::getStringVector(
  const icu::UnicodeString& featureName
) {
  uima::Feature feature = type.getFeatureByBaseName(featureName);
  return utils::arrFStoVec(iterator.get().getStringArrayFSValue(feature));
}


bool AnnotationIterator::isValid() {
  return iterator.isValid();
}


void AnnotationIterator::moveToNext() {
  iterator.moveToNext();
}
