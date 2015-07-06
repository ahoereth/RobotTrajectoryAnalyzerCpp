/**
 * src/StdOutLogger.cpp
 */

#include <string>
#include <iostream>
#include "uima/log.hpp"

class StdCoutLogger : public uima::Logger {
 private:
  bool headline;

 public:
  StdCoutLogger() : headline(true) {}
  explicit StdCoutLogger(bool headline) : headline(headline) {}
  ~StdCoutLogger() {}

  virtual void log(uima::LogStream::EnEntryType entrytype,
                   std::string classname,
                   std::string methodname,
                   std::string message,
                   long errorCode);
};
