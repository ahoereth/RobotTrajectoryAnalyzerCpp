//////
// src/app.cpp

#include "AnnotationGateway.hpp"


/**
 * Main application for running the UIMA annotation flow from a terminal.
 *
 * @param  argc
 * @param  argv
 * @return
 */
int main(int argc, char* argv[]) {
  AnnotationGateway gateway = AnnotationGateway();
  gateway.run();
  return 0;
}
