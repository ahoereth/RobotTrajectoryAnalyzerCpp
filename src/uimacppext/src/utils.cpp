//////
// rta > uimacppext > utils.cpp


#include <string>
#include <vector>
#include <sstream>  // ostringstream, istringstream
#include <cstdlib>  // size_t
#include <cmath>  // pow
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


namespace utils {


/**
 * Basic type conversion.
 *
 * @param  str Input string.
 * @return Output integer.
 */
int toInt(const std::string& str) {
  int num;
  std::istringstream(str) >> num;
  return num;
}


/**
 * Type conversion for vector elements.
 *
 * @param  vec Input vector of string values.
 * @return Resulting vector of integer values.
 */
std::vector<int> toInt(const std::vector<std::string>& vec) {
  std::vector<int> result(vec.size(), 0);
  for (int i = 0, size = vec.size(); i < size; i++) {
    result[i] = toInt(vec[i]);
  }
  return result;
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
 * Convert a C++ standard string to a ICU Unicode String as required by many
 * UIMA applications.
 *
 * @param  str Std string to convert to a unicode string.
 * @return The resulting unicode string.
 */
icu::UnicodeString toUS(const std::string& str) {
  return icu::UnicodeString(str.data(), str.length(), US_INV);
}


/**
 * Split a given source string on a specific delimiter into a vector of
 * substrings.
 *
 * @param  src The source string.
 * @param  del The delimiter to split `src` at.
 * @return Result vector of `src` substrings.
 */
std::vector<std::string> split(const std::string& src, char del) {
  std::vector<std::string> dst;
  std::string str;
  std::istringstream ss(src);
  while (std::getline(ss, str, del)) {
    dst.push_back(str);
  }

  return dst;
}


/**
 * Get the index of a specific value in a vector.
 *
 * @param  vec Vector which is expected to contain the value.
 * @param  val Value to search for.
 * @return Index of `val` in `vec` or `-1` if not found.
 */
int indexOf(
  const std::vector<std::string>& vec,
  const std::string& val
) {
  int index = -1;
  for (int i = 0, size = vec.size(); i < size; ++i) {
    if (vec[i] == val) {
      index = i;
      break;
    }
  }

  return index;
}


/**
 * Calculate the mean of the vector values.
 *
 * @param  vec Vector of double values.
 * @return Mean as double.
 */
double calculateMean(const std::vector<double>& vec) {
  double mean = 0;
  for (size_t i = 0, size = vec.size(); i < size; i++) {
    mean += vec[i] / size;
  }
  return mean;
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
  double mean = calculateMean(vec);
  double variance = 0;
  for (size_t i = 0, size = vec.size(); i < size; i++) {
    variance += pow(mean - vec[i], 2.0) / size;
  }
  return variance;
}


}  // namespace utils
