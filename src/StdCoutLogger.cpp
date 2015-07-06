/**
 * src/StdOutLogger.cpp
 */

#include <string>
#include <iostream>
#include "uima/log.hpp"
#include "StdCoutLogger.hpp"


void StdCoutLogger::log(
  uima::LogStream::EnEntryType enType,
  std::string cpszClass,
  std::string cpszMethod,
  std::string cpszMsg,
  long lUserCode
) {
  if (headline) {
    std::cout << std::endl << "   *** StdCoutLogger ***   " << std::endl;
  }

  if (cpszClass.length() > 0) {
    std::cout << cpszClass << "  :  ";
  }

  if (cpszMethod.length() > 0) {
    std::cout << "Method  : " << cpszMethod << std::endl;
  }

  std::cout << cpszMsg << std::endl;
}
