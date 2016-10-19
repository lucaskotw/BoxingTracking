/**
 * Version: skeleton data only
 * Steps
 * 1) fix writing the skeleton kinematic data
 */
#include <dart/dart.hpp>
#include "MyWindow.h"
#include <iostream>
#include <cmath> // for atan2
#include "AsfParser.h"


using namespace dart;
using namespace dynamics;
using namespace simulation;


int main(int argc, char ** argv)
{

  // TODO: separate asfData for Main.cpp to read
  //
  // testing of reading AsfData
  ASFData* asfData;
  dart::dynamics::SkeletonPtr robot = readSkeleton(argv[1], "robot", &asfData);
  std::vector<std::string> segmentNames;
/*
  if (asfData->getSegmentNames(&segmentNames))
  {
    dtmsg << "[Main.cpp] segmentNames size = "
          << segmentNames.size()
          << std::endl;
  }
*/

  WorldPtr world(new World);

  world->addSkeleton(robot);
  // make sure the world is not intefered with gravity
  Eigen::Vector3d zero_g = Eigen::Vector3d::Zero();
  world->setGravity(zero_g);


  // Display the result

  //MyWindow window(world);
  MyWindow window;
  window.setWorld(world);
  window.setSkel(robot);
  window.loadMotionData(argv[2], asfData);



  // testing with the init pose
  std::vector<std::string> amcSegmentNames = {
    "root",
    "lowerback",
    "upperback",
    "thorax",
    "lowerneck",
    "upperneck",
    "head",
    "rclavicle",
    "rhumerus",
    "rradius",
    "rwrist",
    "rhand",
    "rfingers",
    "rthumb",
    "lclavicle",
    "lhumerus",
    "lradius",
    "lwrist",
    "lhand",
    "lfingers",
    "lthumb",
    "rfemur",
    "rtibia",
    "rfoot",
    "rtoes",
    "lfemur",
    "ltibia",
    "lfoot",
    "ltoes"};
  std::string segmentName;
  std::cout << amcSegmentNames.size() << std::endl;
  for (int i=0; i<amcSegmentNames.size(); ++i)
  {
    segmentName = amcSegmentNames.at(i);
    // set init pose
    window.transformSegmentAtTimeFrame(segmentName, 0);
  }



  glutInit(&argc, argv);
  window.initWindow(800, 800, "Boxing Prototype");
  glutMainLoop();
 
  return 0;

}
