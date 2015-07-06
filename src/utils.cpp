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
 * @param  str Std string to convert to a unicode string.
 * @return The resulting unicode string.
 */
icu::UnicodeString sToUs(const std::string& str) {
  return icu::UnicodeString(str.data(), str.length(), US_INV);
}


/**
 * Basic type conversion.
 *
 * @param  x Float to convert to string.
 * @return The resulting string.
 */
std::string toString(float x) {
  std::ostringstream ss;
  ss << x;
  return ss.str();
}


/**
 * Basic type conversion.
 *
 * @param  x Integer to convert to string.
 * @return The resulting string.
 */
std::string toString(int x) {
  return toString(static_cast<float>(x));
}


/**
 * Basic type conversion.
 *
 * @param  x Unsigned integer to convert to string.
 * @return The resulting string.
 */
std::string toString(std::size_t x) {
  return toString(static_cast<float>(x));
}


}  // namespace utils
