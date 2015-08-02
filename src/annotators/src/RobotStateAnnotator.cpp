//////
// rta > annotators > RobotStateAnnotator.cpp

#include <string>
#include <cstring>  // strcmp
#include <vector>
#include <map>
#include <cstdlib>  // size_t
#include "boost/smart_ptr.hpp"
#include "uima/api.hpp"
#include "urdf/model.h"
#include "unicode/unistr.h"  // UnicodeString
#include "uimautils.hpp"
#include "MongoUrdf.hpp"


using uima::Annotator;  // required for MAKE_AE


class RobotStateAnnotator : public Annotator {
 private:
  uima::LogFacility* log;

  uima::Type JointState;
  uima::Type JointTrajectoryPoint;
  uima::Type DistanceToLimit;
  uima::Type TaskSpacePosition;

  uima::Feature jsNameFtr;   // JointState jointNames
  uima::Feature jsJtpFtr;    // JointState jointTrajectoryPoint
  uima::Feature jtpPosFtr;   // JointTrajectoryPoint positions
  uima::Feature jtpEffFtr;   // JointTrajectoryPoint efforts
  uima::Feature jtpVelFtr;   // JointTrajectoryPoint velocities
  uima::Feature jtpAccFtr;   // JointTrajectoryPoint accelerations
  uima::Feature dstNameFtr;  // DistanceToLimit jointNames
  uima::Feature dstUppFtr;   // DistanceToLimit upperLimits
  uima::Feature dstLowFtr;   // DistanceToLimit lowerLimits
  uima::Feature dstEffFtr;   // DistanceToLimit efforts
  uima::Feature dstVelFtr;   // DistanceToLimit velocities
  uima::Feature tsNameFtr;   // TaskSpacePosition linkName
  uima::Feature tsXyzFtr;    // TaskSpacePosition position
  uima::Feature tsRpyFtr;    // TaskSpacePosition rotation

  // Configuration Parameters
  std::string host;
  std::string database;
  std::string jointsCollection;
  std::string linksCollection;


 public:
  /** Constructor */
  RobotStateAnnotator(void) {}


  /** Destructor */
  ~RobotStateAnnotator(void) {}


  /**
   * Annotator initialization.
   *
   * @param  annotatorContext Interface to the analysis engine's configuration.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId initialize(uima::AnnotatorContext& annotatorContext) {
    log = &annotatorContext.getLogger();
    log->logMessage("RobotStateAnnotator::initialize()");

    // Host ***************************************************
    host = "localhost";
    if (annotatorContext.isParameterDefined("Host")) {
      annotatorContext.extractValue("Host", host);
    }

    // Database ***********************************************
    // Using 'RobotDatabase' because 'Database' is weirdly overwritten by
    // general pipeline.
    database = "pr2";
    if (annotatorContext.isParameterDefined("RobotDatabase")) {
      annotatorContext.extractValue("RobotDatabase", database);
    }

    // JointsCollection ***************************************
    jointsCollection = "robot_joints";
    if (annotatorContext.isParameterDefined("JointsCollection")) {
      annotatorContext.extractValue("JointsCollection", jointsCollection);
    }

    // LinksCollection ****************************************
    linksCollection = "robot_links";
    if (annotatorContext.isParameterDefined("LinksCollection")) {
      annotatorContext.extractValue("LinksCollection", linksCollection);
    }

    log->logMessage("Host: " + host + ", Database: " + database + ", "
                    "JointsCollection: " + jointsCollection + ", "
                    "LinksCollection: " + linksCollection);
    return UIMA_ERR_NONE;
  }


  /**
   * Type system initialization.
   *
   * Types:
   *   * JointState
   *   * JointTrajectoryPoint
   *   * DistanceToLimit
   *   * TaskSpacePosition
   *
   * @param  typeSystem The container of all types available through the AE.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId typeSystemInit(const uima::TypeSystem& typeSystem) {
    log->logMessage("RobotStateAnnotator::typeSystemInit()");

    // JointState *********************************************
    JointState = typeSystem.getType("JointState");
    if (!JointState.isValid()) {
      log->logError("Error getting Type object for JointState");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jsNameFtr = JointState.getFeatureByBaseName("jointNames");
    jsJtpFtr  = JointState.getFeatureByBaseName("jointTrajectoryPoint");

    // JointTrajectoryPoint ***********************************
    JointTrajectoryPoint = typeSystem.getType("JointTrajectoryPoint");
    if (!JointTrajectoryPoint.isValid()) {
      log->logError("Error getting Type object for JointTrajectoryPoint");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    jtpPosFtr = JointTrajectoryPoint.getFeatureByBaseName("positions");
    jtpEffFtr = JointTrajectoryPoint.getFeatureByBaseName("efforts");
    jtpVelFtr = JointTrajectoryPoint.getFeatureByBaseName("velocities");
    jtpAccFtr = JointTrajectoryPoint.getFeatureByBaseName("accelerations");

    // DistanceToLimit ****************************************
    DistanceToLimit = typeSystem.getType("DistanceToLimit");
    if (!DistanceToLimit.isValid()) {
      log->logError("Error getting Type object for DistanceToLimit");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    dstNameFtr = DistanceToLimit.getFeatureByBaseName("jointNames");
    dstUppFtr  = DistanceToLimit.getFeatureByBaseName("upperLimits");
    dstLowFtr  = DistanceToLimit.getFeatureByBaseName("lowerLimits");
    dstEffFtr  = DistanceToLimit.getFeatureByBaseName("efforts");
    dstVelFtr  = DistanceToLimit.getFeatureByBaseName("velocities");

    // TaskSpacePosition **************************************
    TaskSpacePosition = typeSystem.getType("TaskSpacePosition");
    if (!TaskSpacePosition.isValid()) {
      log->logError("Error getting Type object for TaskSpacePosition");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    tsNameFtr = TaskSpacePosition.getFeatureByBaseName("linkName");
    tsXyzFtr  = TaskSpacePosition.getFeatureByBaseName("position");
    tsRpyFtr  = TaskSpacePosition.getFeatureByBaseName("rotation");

    return UIMA_ERR_NONE;
  }


  /**
   * Clean up on annotator destruction.
   *
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId destroy() {
    log->logMessage("RobotStateAnnotator::destroy()");
    return UIMA_ERR_NONE;
  }


  /**
   * TaskSpacePosition annotations.
   *
   * @return UIMA error id. UIMA_ERR_NONE on success.
   */
  uima::TyErrorId annotateTaskSpace(
    uima::CAS& cas,
    const urdf::Model& model
  ) {
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::FeatureStructure ts;

    boost::shared_ptr<urdf::Link> link;
    std::vector< boost::shared_ptr<urdf::Link> > links;
    model.getLinks(links);

    for (std::size_t i = 0, size = links.size(); i < size; i++) {
      link = links[i];
      ts = cas.createFS(TaskSpacePosition);
      ts.setFSValue(tsXyzFtr,
        utils::toDoubleArrayFS(cas, MongoUrdf::getPosition(link)));
      ts.setFSValue(tsRpyFtr,
        utils::toDoubleArrayFS(cas, MongoUrdf::getRotation(link)));
      index.addFS(ts);
    }

    return UIMA_ERR_NONE;
  }


  /**
   * DistanceToLimit annotations.
   *
   * @return UIMA error id. UIMA_ERR_NONE on success.
   */
  uima::TyErrorId annotateLimits(
    uima::CAS& cas,
    const urdf::Model& model
  ) {
    uima::FSIndexRepository& index = cas.getIndexRepository();
    uima::ANIndex jsIndex = cas.getAnnotationIndex(JointState);
    uima::ANIterator jsIter = jsIndex.iterator();

    uima::AnnotationFS js, dst;
    uima::FeatureStructure jtp;

    uima::StringArrayFS jsNames;
    uima::DoubleArrayFS jtpPositions, jtpVelocities, jtpEfforts;

    boost::shared_ptr<const urdf::Joint> joint;
    boost::shared_ptr<urdf::JointLimits> limits;

    std::vector<std::string> dstNames;
    std::vector<double> upperLimits, lowerLimits, velocities, efforts;

    while (jsIter.isValid()) {
      js = jsIter.get();
      jtp = js.getFSValue(jsJtpFtr);

      // Get feature structure value arrays.
      jsNames       = js.getStringArrayFSValue(jsNameFtr);
      jtpPositions  = jtp.getDoubleArrayFSValue(jtpPosFtr);
      jtpVelocities = jtp.getDoubleArrayFSValue(jtpEffFtr);
      jtpEfforts    = jtp.getDoubleArrayFSValue(jtpVelFtr);

      // Clear storage vectors.
      dstNames.clear();
      upperLimits.clear();
      lowerLimits.clear();
      velocities.clear();
      efforts.clear();

      for (std::size_t i = 0; i < jsNames.size(); i++) {
        joint = model.getJoint(jsNames.get(i).asUTF8());
        limits = joint->limits;

        // Limits are only relevant for some joint types.
        if (
          joint->type == urdf::Joint::REVOLUTE ||
          joint->type == urdf::Joint::PRISMATIC
        ) {
          dstNames.push_back(joint->name);
          upperLimits.push_back(limits->upper - jtpPositions.get(i));
          lowerLimits.push_back(limits->lower - jtpPositions.get(i));
          velocities.push_back(limits->velocity - jtpVelocities.get(i));
          efforts.push_back(limits->effort - jtpEfforts.get(i));
        }
      }

      dst = cas.createAnnotation(DistanceToLimit,
                                 js.getBeginPosition(), js.getEndPosition());
      dst.setFSValue(dstNameFtr, utils::toStringArrayFS(cas, dstNames));
      dst.setFSValue(dstUppFtr, utils::toDoubleArrayFS(cas, upperLimits));
      dst.setFSValue(dstLowFtr, utils::toDoubleArrayFS(cas, lowerLimits));
      dst.setFSValue(dstVelFtr, utils::toDoubleArrayFS(cas, velocities));
      dst.setFSValue(dstEffFtr, utils::toDoubleArrayFS(cas, efforts));

      index.addFS(dst);
      jsIter.moveToNext();
    }

    return UIMA_ERR_NONE;
  }


  /**
   * Data processing.
   *
   * @param  cas                 The current common analysis system.
   * @param  resultSpecification The specification of expected results as given
   *                             by the annotator descriptor.
   * @return UIMA error type id - UIMA_ERR_NONE on success.
   */
  uima::TyErrorId process(
    uima::CAS& cas,
    const uima::ResultSpecification& resultSpecification
  ) {
    log->logMessage("RobotStateAnnotator::process() begins");
    uima::TyErrorId errorId = UIMA_ERR_NONE;

    // Parse MongoDB data into an urdf model.
    MongoUrdf* urdf = new MongoUrdf(host);
    urdf::Model model = urdf->getModel(
      database, linksCollection, jointsCollection);
    delete urdf;

    // Process robot links - TaskSpacePosition annotations.
    errorId = annotateTaskSpace(cas, model);
    if (UIMA_ERR_NONE != errorId) {
      return errorId;
    }

    // Process robot joints - DistanceToLimit annotations.
    errorId = annotateLimits(cas, model);

    log->logMessage("RobotStateAnnotator::process() ends");
    return UIMA_ERR_NONE;
  }
};


MAKE_AE(RobotStateAnnotator);
