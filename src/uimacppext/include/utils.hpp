//////
// rta > uimacppext > utils.hpp

#ifndef __UTILS_INCLUDE__
#define __UTILS_INCLUDE__


#include <string>
#include <vector>
#include <sstream>  // ostringstream, istringstream
#include <cstdlib>  // size_t
#include <cmath>  // pow
#include <algorithm>  // transform
#include "unicode/unistr.h"  // UnicodeString


/**
 * General C++ utility functions.
 */
namespace utils {


int toInt(const std::string& str);
std::vector<int> toInt(const std::vector<std::string>& vec);
std::string toString(double x);
std::string toString(float x);
std::string toString(int x);
std::string toString(std::size_t x);
std::string toString(const icu::UnicodeString& us);
std::vector<std::string> toString(const std::vector<icu::UnicodeString>& vec);
std::vector<std::string> toString(const std::vector<double>& vec);
std::vector<std::string> toString(const std::vector<int>& vec);
icu::UnicodeString toUS(const std::string& str);
std::vector<std::string> split(const std::string& src, char del);
std::string join(const std::vector<std::string>& src, const std::string& sep);
std::string join(const std::vector<double>& src, const std::string& sep);
std::string join(const std::vector<int>& src, const std::string& sep);
std::vector<std::string> sub(const std::vector<std::string>& vec,
                             const std::vector<int>& indices);
std::vector<int> range(int begin, int end, int stepsize = 1);
int indexOf(const std::vector<std::string>& vec, const std::string& val);
double calculateMean(const std::vector<double>& vec);
double calculateVariance(const std::vector<double>& vec);
std::string toLower(std::string str);


}  // namespace utils


#endif
