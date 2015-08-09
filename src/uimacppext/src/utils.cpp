//////
// rta > uimacppext > utils.cpp


#include <string>
#include <vector>
#include <sstream>  // ostringstream, istringstream
#include <cstdlib>  // size_t
#include <cmath>  // pow
#include <algorithm>  // transform
#include "unicode/unistr.h"  // UnicodeString
#include "utils.hpp"


namespace utils {


////////////////////////////////////////////////////////////////////////////////
/// Numeric Type Conversion Functions.

/**
 * Type conversion.
 *
 * @param  str Standard string.
 * @return Integer.
 */
int toInt(const std::string& str) {
  int num;
  std::istringstream(str) >> num;
  return num;
}


/**
 * Type conversion for vector elements.
 *
 * @param  vec Vector of standard strings.
 * @return Vector of integers.
 */
std::vector<int> toInt(const std::vector<std::string>& vec) {
  std::vector<int> result(vec.size(), 0);
  for (int i = 0, size = vec.size(); i < size; i++) {
    result[i] = toInt(vec[i]);
  }
  return result;
}




////////////////////////////////////////////////////////////////////////////////
/// String Type Conversion Functions.

/**
 * Type conversion.
 *
 * @param  x Double.
 * @return Standard string.
 */
std::string toString(double x) {
  std::ostringstream ss;
  ss << x;
  return ss.str();
}


/**
 * Type conversion.
 *
 * @param  x Float.
 * @return Standard string.
 */
std::string toString(float x) {
  return toString(static_cast<double>(x));
}


/**
 * Type conversion.
 *
 * @param  x Integer.
 * @return Standard string.
 */
std::string toString(int x) {
  return toString(static_cast<double>(x));
}


/**
 * Type conversion.
 *
 * @param  x Unsigned integer.
 * @return Standard string.
 */
std::string toString(std::size_t x) {
  return toString(static_cast<double>(x));
}


/**
 * Type conversion.
 *
 * @param  us Unicode string.
 * @return The resulting string.
 */
std::string toString(const icu::UnicodeString& us) {
  char* cs = new char;
  us.extract(0, us.length(), cs);
  std::string str(cs);
  delete cs;
  return str;
}


/**
 * Type conversion for vector elements.
 *
 * @param  vec Vector of unicode strings.
 * @return Vector of standard strings.
 */
std::vector<std::string> toString(const std::vector<icu::UnicodeString>& vec) {
  std::vector<std::string> result(vec.size());
  std::vector<icu::UnicodeString>::const_iterator it;
  std::size_t i = 0;
  for (it = vec.begin(), i = 0; vec.end() != it; it++, i++) {
    result[i] = toString(*it);
  }
  return result;
}


/**
 * Type conversion for vector elements.
 *
 * @param  vec Vector of doubles.
 * @return Vector of standard strings.
 */
std::vector<std::string> toString(const std::vector<double>& vec) {
  std::vector<std::string> result(vec.size());
  std::vector<double>::const_iterator it;
  std::size_t i;
  for (it = vec.begin(), i = 0; vec.end() != it; it++, i++) {
    result[i] = toString(*it);
  }
  return result;
}


/**
 * Type conversion for vector elements.
 *
 * @param  vec Vector of integers.
 * @return Vector of standard strings.
 */
std::vector<std::string> toString(const std::vector<int>& vec) {
  std::vector<std::string> result(vec.size());
  std::vector<int>::const_iterator it;
  std::size_t i;
  for (it = vec.begin(), i = 0; vec.end() != it; it++, i++) {
    result[i] = toString(*it);
  }
  return result;
}




/**
 * Type conversion
 *
 * @param  str Standard string.
 * @return Unicode string.
 */
icu::UnicodeString toUS(const std::string& str) {
  return icu::UnicodeString(str.data(), str.length(), US_INV);
}




////////////////////////////////////////////////////////////////////////////////
/// String Manipulation Functions.

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
 * Concatenates a given source vector's elements into a single string with
 * `sep` as seperator between each value.
 *
 * @param  vec The source vector.
 * @param  del The seperator to insert inbetween the vector's values.
 * @return Result string of concatenated `src` values.
 */
std::string join(const std::vector<std::string>& vec, const std::string& sep) {
  std::string str = "";
  int size = vec.size(), i = 0;

  for (
    std::vector<std::string>::const_iterator it = vec.begin();
    vec.end() != it; it++, i++
  ) {
    str += *it;

    // Seperator.
    if (1+i < size) {
      str += sep;
    }
  }

  return str;
}


/**
 * Concatenates a given source vector's elements into a single string with
 * `sep` as seperator between each value.
 *
 * @param  vec The source vector.
 * @param  del The seperator to insert inbetween the vector's values.
 * @return Result string of concatenated `src` values.
 */
std::string join(const std::vector<double>& vec, const std::string& sep) {
  return join(toString(vec), sep);
}


/**
 * Concatenates a given source vector's elements into a single string with
 * `sep` as seperator between each value.
 *
 * @param  vec The source vector.
 * @param  del The seperator to insert inbetween the vector's values.
 * @return Result string of concatenated `src` values.
 */
std::string join(const std::vector<int>& vec, const std::string& sep) {
  return join(toString(vec), sep);
}


/**
 * Converts a given string to lowercase.
 *
 * @param  {std::string} str UPPERcase string.
 * @return {std::string} lowercase string.
 */
std::string toLower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  return str;
}




////////////////////////////////////////////////////////////////////////////////
/// Vector Manipulation Functions.

/**
 * Get a sub vector from the source vector as specified by a vector of indicies.
 *
 * @param vec
 * @param indices
 * @result
 */
std::vector<std::string> sub(
  const std::vector<std::string>& vec,
  const std::vector<int>& indices
) {
  std::vector<std::string> result(indices.size());
  int i = 0;

  for (
    std::vector<int>::const_iterator it = indices.begin();
    it != indices.end(); it++
  ) {
    result[i++] = vec[*it];
  }

  return result;
}


/**
 * Generate a vector of integers which inclusivley range from begin to end.
 *
 * @param begin    Integer to begin the range at.
 * @param end      Integer to end the range at.
 * @param stepsize Positive integer defining the stepsize. Ascending/decreasing
 *                 is determined automatically from the relation of the begin
 *                 and end parameters. (default: 1)
 */
std::vector<int> range(int begin, int end, int stepsize) {
  int step = ((begin < end) ? 1 : -1) * stepsize;
  int size = std::abs(end-begin)+1;  // + 1 because begin AND end are included.
  std::vector<int> result(size);
  for (int i = 0; i < size; i += step) {
    result[i] = i+begin;
  }
  return result;
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




////////////////////////////////////////////////////////////////////////////////
/// Numeric Manipulation Functions.

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
