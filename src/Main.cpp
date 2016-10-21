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
#if 0
  // testing with init post of upperbody
 /* 
  Eigen::Vector3d segmentAxes;
  double deg_to_rad = M_PI/180;
  Eigen::Matrix3d refFrame = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d motionRot = Eigen::Matrix3d::Identity();
  Eigen::Isometry3d finalTF = Eigen::Isometry3d::Identity();
  // lowerback
  asfData->getSegmentAxes("lowerback", &segmentAxes);
  refFrame = dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
  motionRot = dart::math::eulerZYXToMatrix(
      Eigen::Vector3d(8.50507, -1.64919, -0.803095)*deg_to_rad);
  finalTF.translation() =
  robot->getBodyNode("lowerback")->getParentJoint()
       ->getTransformFromParentBodyNode().translation();
  finalTF.linear() = refFrame * motionRot * refFrame.inverse();
  robot->getBodyNode("lowerback")->getParentJoint()
       ->setTransformFromParentBodyNode(finalTF);

  // upperback
  asfData->getSegmentAxes("upperback", &segmentAxes);
  refFrame = dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
  motionRot = dart::math::eulerZYXToMatrix(
      Eigen::Vector3d(-0.253411, -2.07223, 3.99623)*deg_to_rad);
  finalTF = Eigen::Isometry3d::Identity();
  finalTF.translation() =
    robot->getBodyNode("upperback")->getParentJoint()
         ->getTransformFromParentBodyNode().translation();
  finalTF.linear() = refFrame * motionRot * refFrame.inverse();
  robot->getBodyNode("upperback")->getParentJoint()
       ->setTransformFromParentBodyNode(finalTF);
  
  // thorax
  asfData->getSegmentAxes("thorax", &segmentAxes);
  refFrame = dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
  motionRot = dart::math::eulerZYXToMatrix(
      Eigen::Vector3d(-4.86222, -0.95435, 4.89323)*deg_to_rad);
      //Eigen::Vector3d(0, 0, 0)*deg_to_rad);
  finalTF = Eigen::Isometry3d::Identity();
  finalTF.translation() =
    robot->getBodyNode("thorax")->getParentJoint()
         ->getTransformFromParentBodyNode().translation();
  finalTF.linear() = refFrame * motionRot * refFrame.inverse();
  robot->getBodyNode("thorax")->getParentJoint()
       ->setTransformFromParentBodyNode(finalTF);
  // lclavicle
  asfData->getSegmentAxes("lclavicle", &segmentAxes);
  refFrame = dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
  motionRot = dart::math::eulerZYXToMatrix(
      Eigen::Vector3d(0, -1.758e-014, -1.07344e-014)*deg_to_rad);
      //Eigen::Vector3d(0, 90, 0)*deg_to_rad);
  finalTF = Eigen::Isometry3d::Identity();
  finalTF.translation() =
    robot->getBodyNode("lclavicle")->getParentJoint()
         ->getTransformFromParentBodyNode().translation();
  finalTF.linear() = refFrame.inverse() * motionRot * refFrame;
  robot->getBodyNode("lclavicle")->getParentJoint()
       ->setTransformFromParentBodyNode(finalTF);

 // lhumerus
  asfData->getSegmentAxes("lhumerus", &segmentAxes);

  finalTF = Eigen::Isometry3d::Identity();
  refFrame = dart::math::eulerZYXToMatrix(
      Eigen::Vector3d(segmentAxes(2), segmentAxes(1), segmentAxes(0))*deg_to_rad);
  motionRot = dart::math::eulerZYXToMatrix(
      //Eigen::Vector3d(-21.6898, -14.6973, 90.1038)*deg_to_rad);
      Eigen::Vector3d(90.1038, -14.6973, -21.6898)*deg_to_rad);
      //Eigen::Vector3d(90, 0, 0)*deg_to_rad);

  //std::string parentName;
  //asfData->getSegmentParentName("lhumerus", &parentName);
  //Eigen::Vector3d parentAxes;
  //asfData->getSegmentAxes(parentName, &parentAxes);
  //refFrame = dart::math::eulerZYXToMatrix(parentAxes*deg_to_rad).transpose()
  //            * dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
  //refFrame = dart::math::eulerZYXToMatrix(parentAxes*deg_to_rad);
              * dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
 
  finalTF.linear() = refFrame * motionRot * refFrame.inverse();

  //finalTF.linear() = refFrame;
  finalTF.translation() =
    robot->getBodyNode("lhumerus")->getParentJoint()
         ->getTransformFromParentBodyNode().translation();
  robot->getBodyNode("lhumerus")->getParentJoint()
       ->setTransformFromParentBodyNode(finalTF);
  //robot->getJoint("lhumerus_joint")
  //     ->setPositions(dart::dynamics::BallJoint::convertToPositions(finalTF.linear()));
  // lradius
  asfData->getSegmentAxes("lradius", &segmentAxes);
  refFrame = dart::math::eulerZYXToMatrix(
      Eigen::Vector3d(segmentAxes(2), segmentAxes(1), segmentAxes(0))*deg_to_rad);
 
//  refFrame = dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
 
  motionRot = dart::math::eulerZYXToMatrix(
      //Eigen::Vector3d(92.7969, 0, 0)*deg_to_rad);
      Eigen::Vector3d(0, 0, 92.7969)*deg_to_rad);
  finalTF = Eigen::Isometry3d::Identity();
  finalTF.translation() =
    robot->getBodyNode("lradius")->getParentJoint()
         ->getTransformFromParentBodyNode().translation();
  finalTF.linear() = refFrame * motionRot * refFrame.inverse();
  //finalTF.linear() =  motionRot;
  robot->getBodyNode("lradius")->getParentJoint()
       ->setTransformFromParentBodyNode(finalTF);
  dtmsg << "[Main.cpp] lradius's relative coord"
        << "\nsegmentAxes = " << segmentAxes
        << "\nAxes Rotation Matrix" << dart::math::eulerXYZToMatrix(segmentAxes)
        << "\nMotion Rotation Matrix" << dart::math::eulerXYZToMatrix(
      Eigen::Vector3d(0, 0, 92.7969)*deg_to_rad)
        << std::endl;
*/
#else

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
#endif
  glutInit(&argc, argv);
  window.initWindow(800, 800, "Boxing Prototype");
  glutMainLoop();
  return 0;

}
