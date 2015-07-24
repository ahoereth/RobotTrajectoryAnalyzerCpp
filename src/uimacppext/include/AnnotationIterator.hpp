//////
// rta > uimacppext > AnnotationIterator.hpp

#ifndef __AnnotationIterator_INCLUDE__
#define __AnnotationIterator_INCLUDE__


#include <string>
#include <vector>
#include "uima/api.hpp"
#include "unicode/unistr.h"  // UnicodeString
#include "uimautils.hpp"


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
  std::vector<double> getDoubleVector(
    const icu::UnicodeString& featureName);
  std::vector<std::string> getStringVector(
    const icu::UnicodeString& featureName);
  bool isValid();
  void moveToNext();
};


#endif  // ifndef __ANNOTATIONGATEWAY_INCLUDE__
