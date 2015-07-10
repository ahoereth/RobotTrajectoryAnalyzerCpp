//////
// rta > uimacppext > AnnotationStream.cpp

#include <vector>
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "AnnotationIterator.hpp"
#include "utils.hpp"


AnnotationIterator::AnnotationIterator(
  const uima::ANIterator& iterator,
  const uima::Type& type
) : iterator(iterator), type(type) {}


uima::AnnotationFS AnnotationIterator::pop() {
  uima::AnnotationFS fs;
  if (iterator.isValid()) {
    fs = iterator.get();
    iterator.moveToNext();
  }

  return fs;
}


std::vector<double> AnnotationIterator::getDoubleVector(
  const icu::UnicodeString& featureName
) {
  uima::Feature feature = type.getFeatureByBaseName(featureName);
  return utils::arrFStoVec(iterator.get().getDoubleArrayFSValue(feature));
}


bool AnnotationIterator::isValid() {
  return iterator.isValid();
}


void AnnotationIterator::moveToNext() {
  iterator.moveToNext();
}
