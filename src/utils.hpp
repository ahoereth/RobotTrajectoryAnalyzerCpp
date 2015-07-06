/**
 * src/utils.hpp
 */

#include <string>
#include <sstream>
#include <cstdlib>
#include "unicode/unistr.h"

/**
 * General C++ helper functions. Some of those are reimplementations of
 * functions which are part of the C++11 standard.
 */
namespace utils {

icu::UnicodeString sToUs(const std::string& str);

}  // namespace utils
