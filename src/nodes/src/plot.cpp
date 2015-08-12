//////
// rta > nodes > acceleration_topic.cpp
//
// The do it all executable for running the annotation pipeline and firing
// off the data visualizable with rqt_plot.

#include <string>
#include <vector>
#include <cstring>
#include <cstdio>  // printf, puts
#include <utility>  // pair, make_pair
#include "unicode/unistr.h"  // UnicodeString"
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "uimautils.hpp"
#include "AnnotationGateway.hpp"
#include "Topics.hpp"


////////////////////////////////////////////////////////////////////////////////
// Helper.

/**
 * Print a list to cout with specified widths.
 */
void coutlist(
  const std::vector<std::string>& elements,
  bool showindex = true,
  int width = 40
) {
  std::vector<std::string>::const_iterator it;
  std::size_t i = 0;

  if (showindex) {
    width -= 5;
  }

  for (it = elements.begin(); it != elements.end(); it++, i++) {
    // Index.
    if (showindex) {
      std::printf("%3zu) ", i);
    }

    // String.
    std::printf("%-*s", width, it->c_str());

    // Linebreak.
    if (i%2 == 1 || it+1 == elements.end()) {
      std::puts("");
    }
  }
}


/**
 * 80 character line seperator for CUI.
 */
void coutlinesep() {
  std::puts("----------------------------------------"
            "----------------------------------------");
}


/**
 * Display a list of joints and provider the user with the option to select
 * from them.
 */
std::pair< const std::vector<int>, const std::vector<std::string> > chooseJoints(
  const std::vector<std::string>& names
) {
  coutlinesep();
  coutlist(names);

  // Prompt to make a choice.
  std::printf("%s", "Comma seperated list of joint indices (all): ");
  std::string choice;
  std::getline(std::cin, choice);

  // Get choosen joints and show them to the user.
  std::vector<int> indices = utils::toInt(utils::split(choice, ','));
  if (indices.size() == 0) {
    indices = utils::range(0, names.size()-1);
  }
  std::vector<std::string> selected = utils::sub(names, indices);
  coutlist(selected, false);

  return std::make_pair(indices, selected);
}


/**
 * Let the user choose which view to plot.
 */
std::string choosePlot(int size) {
  std::vector<std::string> plots = Topics::getPlots();
  std::string tmp;

  // List of available plots.
  coutlinesep();
  coutlist(plots);

  // Plot choice.
  std::printf("%s", "Choose a single view to plot specified by it's index: ");
  std::size_t choice;
  do {
    std::getline(std::cin, tmp);
    choice = utils::toInt(tmp);
    if (choice < 0 || choice >= plots.size()) {
      std::puts("Specify a single integer index!");
    }
  } while (choice < 0 || choice >= plots.size());

  // Plotting command.
  coutlinesep();
  std::puts("For visualization execute the following command "
            "in a seperate terminal.\n");
  std::cout << Topics::getCommand(plots[choice], size) << std::endl;
  std::cout << std::endl << "Press any key to start publishing...";
  std::getline(std::cin, tmp);
  return plots[choice];
}


/**
 *
 *//*
void movblub() {

    int now = jsIter.getString("time");

    // Start publishing.
    while (node.ok()) {
      if (posIter.get().getEndPosition() > now) {
        movIter.moveToNext();
      }

      if (velIter.get().getEndPosition() > now) {
        velIter.moveToNext();
      }
}*/



////////////////////////////////////////////////////////////////////////////////
// Main.

/**
 * Putting it all together.
 *
 * @param  argc
 * @param  argv
 * @return
 */
int main(int argc, char* argv[]) {
  // Default options.
  bool loop = false;
  bool echo = false;
  int rate = 1000;

  // Parse command line arguments.
  for (int i = 0; i < argc; i++) {
    if (0 == std::strcmp(argv[i], "-loop")) {
      loop = true;
    } else if (0 == std::strcmp(argv[i], "-echo")) {
      echo = true;
    } else if (0 == std::strncmp(argv[i], "-rate=", 6)) {
      rate = utils::toInt(std::string(argv[i]).substr(6));
    }
  }

  // Initalize and run annotators
  AnnotationGateway gateway = AnnotationGateway();
  gateway.run();

  // Get required iterators.
  uima::Feature jnFtr = gateway.getFeature("JointState", "jointNames");
  uima::ANIterator jsIter = gateway.getANIterator("JointState");
  std::vector<std::string> jointNames;
  jointNames = utils::toVector(jsIter.get().getStringArrayFSValue(jnFtr));

  // Let user choose which joints to display.
  std::pair< std::vector<int>, std::vector<std::string> > selected;
  selected = chooseJoints(jointNames);

  std::string plot = choosePlot(selected.first.size());
  coutlinesep();

  // Initialize ros.
  ros::init(argc, argv, "rta_plotting");
  ros::NodeHandle node;

  // Fire off the topic!
  Topics topics = Topics(node, gateway, selected, rate, loop, echo);
  topics.plot(plot);

  return 0;
}
