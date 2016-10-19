#include "MyWindow.h"
#include <unistd.h> // pause for a second


const double deg_to_rad = M_PI/180.0;
const double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter


/*
bool transformSegment(std::string segmentName,
                      Eigen::VectorXd segmentConfig)
{

}
*/


void MyWindow::setSkel(dart::dynamics::SkeletonPtr _skel)
{
  mSkel = _skel;

  // set initial position
}



int MyWindow::loadMotionData(char* motionFileName, ASFData * asfData)
{

    // load corresponding ASF data
    mAsfData = new ASFData();
    *mAsfData = *asfData;

//    std::cout << "MyWindow get Asf Data pointer" << std::endl;    
    // test motion data
    mInputMotion = new AMCData(asfData);

//    std::cout << mInputMotion->getSegmentNames().size() << std::endl;    
    mInputMotion->readAMCFile(motionFileName);
    
    return 0;
}

bool MyWindow::transformSegmentAtTimeFrame(std::string segmentName,
                                           int timeStep)
{
  std::cout << segmentName << std::endl;
  Eigen::VectorXd segmentConfig;
  if (mInputMotion->getSegmentConfig(timeStep, segmentName, &segmentConfig))
  {
    if (segmentName == "root")
    {

      // Transform Root
      Eigen::Vector6d r_t = segmentConfig;

      // r_t:       head(3)->translation, tail(3)->rotation
      // FreeJoint: head(3)->rotation, tail(3)->translation

      Eigen::Isometry3d tf;
      tf.linear() = dart::math::eulerXYZToMatrix(r_t.tail(3)*deg_to_rad);
      //tf.linear() = dart::math::eulerZYXToMatrix(r_t.tail(3)*deg_to_rad);
      tf.translation() = r_t.head(3)*unit;
      dtmsg << "[MyWindow::transformSegmentAtTimeFrame]"
            << " segment name = " << segmentName
            << " segment configuration = " << segmentConfig
            << " convert to positions with size = " << dart::dynamics::FreeJoint::convertToPositions(tf).size()
            << " to joint with #dofs = " << mSkel->getJoint("root_joint")->getNumDofs()
            << std::endl;
      mSkel->getJoint("root_joint")
           ->setPositions(dart::dynamics::FreeJoint::convertToPositions(tf));

      dtmsg << "[MyWindow::transformSegmentAtTimeFrame]"
            << " segment name = " << segmentName
            << " segment configuration = "
            << mSkel->getJoint("root_joint")->getPositions()
            << " after setting"
            << std::endl;
 
      return true;
    }
    else
    {
      // other bones
      // prepare reference frame
      Eigen::Vector3d segmentAxes;
      if (mAsfData->getSegmentAxes(segmentName, &segmentAxes))
      {
        Eigen::Matrix3d refFrame = dart::math::eulerXYZToMatrix(segmentAxes*deg_to_rad);
        //Eigen::Matrix3d refFrame = dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
        // calculate motion transformation
        Eigen::Matrix3d motionRot = dart::math::eulerXYZToMatrix(segmentConfig*deg_to_rad);
        //Eigen::Matrix3d motionRot = dart::math::eulerZYXToMatrix(segmentConfig*deg_to_rad);
        Eigen::Isometry3d finalTF;
        //finalTF.linear() = refFrame.inverse() * motionRot * refFrame;
        finalTF.linear() = refFrame.transpose() * motionRot * refFrame;
        //finalTF.linear() = refFrame * motionRot * refFrame.inverse();
        //finalTF.linear() = motionRot;

        // transform
        mSkel->getBodyNode(segmentName)->getParentJoint()
             ->setPositions(dart::dynamics::BallJoint::convertToPositions(finalTF.linear()));


        //std::cout << segmentName << std::endl;
        //std::cout << mSkel->getBodyNode(segmentName)->getParentJoint()
        //    ->getName() << std::endl;
        //std::cout << mSkel->getBodyNode(segmentName)->getRelativeTransform().linear()
        //          << std::endl;


        //std::cout << mSkel->getJoint(segmentName+"_joint")->getPositions() << std::endl;

        return true;

      }
      std::cout << "segment transform fail" << std::endl;
      return false;
    }
  }
  return false;
}

void MyWindow::timeStepping()
{


  // Step the simulation forward
  mWorld->step();

  /*
  std::vector<std::string> segmentNames = mInputMotion->getSegmentNames();
  std::cout << segmentNames.size() << std::endl;
  std::cout << segmentNames.at(10) << std::endl;
  std::string segmentName;
  for (int i=0; i<segmentNames.size(); ++i)
  {
    segmentName = segmentNames.at(i);
    // set init pose
    transformSegmentAtTimeFrame(segmentName, 0);
  }
  */


}
