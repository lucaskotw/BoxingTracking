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

  // testing of reading AsfData
  ASFData* asf_data = new ASFData();
  dart::dynamics::SkeletonPtr robot = dart::dynamics::Skeleton::create("robot");
  robot = readSkeleton(argv[1]);

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


  glutInit(&argc, argv);
  window.initWindow(800, 800, "Boxing Prototype");
  glutMainLoop();
 
  return 0;

}
