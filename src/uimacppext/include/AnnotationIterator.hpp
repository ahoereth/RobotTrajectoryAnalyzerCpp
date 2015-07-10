//////
// rta > uimacppext > AnnotationIterator.hpp

#ifndef __AnnotationIterator_INCLUDE__
#define __AnnotationIterator_INCLUDE__


#include <vector>
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


/**
 *
 */
class AnnotationIterator {
 private:
  uima::ANIterator iterator;
  uima::Type type;

 public:
  explicit AnnotationIterator(const uima::ANIterator& iterator,
                              const uima::Type& type);
  ~AnnotationIterator(void) {}
  uima::AnnotationFS pop();
  std::vector<double> getDoubleVector(
    const icu::UnicodeString& featureName);
  bool isValid();
  void moveToNext();
};


#endif  // ifndef __ANNOTATIONGATEWAY_INCLUDE__
