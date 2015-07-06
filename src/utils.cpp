/**
 * src/utils.cpp
 */

#include <string>
#include <sstream>
#include <cstdlib>  // size_t
#include "unicode/unistr.h"
#include "utils.hpp"


namespace utils {


/**
 * Convert a C++ standard string to a ICU Unicode String as required by many
 * UIMA applications.
 *
 * @param  {std::string} str
 * @return {icu::UnicodeString}
 */
icu::UnicodeString sToUs(const std::string& str) {
  return icu::UnicodeString(str.data(), str.length(), US_INV);
}


}  // namespace utils
