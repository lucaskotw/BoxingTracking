/*
 * DART Version: 6.1
 *
 */
#include "AsfParser.h"
#include <iostream> // demonstrate the result



using namespace dart::dynamics;
using namespace dart::simulation;


/* Constructor and Destructor */
ASFData::ASFData()
{

}

ASFData::ASFData(const ASFData & asfData) : mRoot(asfData.mRoot),
                                            mSegments(asfData.mSegments)
{
}


ASFData& ASFData::operator= (const ASFData & asfData)
{
  mRoot = asfData.mRoot;
  mSegments = asfData.mSegments;
  return *this;

}


ASFData::~ASFData()
{}


/* 
 * getter for a specific Segment, which are supposed to be accessed only by
 * instance itself
 */
Segment * ASFData::getSegmentPtr(std::string segmentName)
{
  for (int i=0; i<mSegments.size(); ++i)
  {
    if (mSegments.at(i).name == segmentName)
      return &mSegments.at(i);
  }
  return nullptr;
}



// ============================================================================
//   getter for segment
// ============================================================================
Eigen::Vector3d ASFData::getSegmentDirection(std::string segmentName)
{
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  for (int i=0; i<mSegments.size(); ++i)
  {
    if (mSegments.at(i).name == segmentName)
    {
      direction = mSegments.at(i).direction;
    }
  }
  return direction;

}


double ASFData::getSegmentLength(std::string segmentName)
{
  double length = 0.0;
  for (int i=0; i<mSegments.size(); ++i)
  {
    if (mSegments.at(i).name == segmentName)
    {
      length = mSegments.at(i).length;
    }
  }
  return length;

}

bool ASFData::getSegmentAxes(std::string segmentName,
                             Eigen::Vector3d * segmentAxes)
{
  for (int i=0; i<mSegments.size(); ++i)
  {
    if (mSegments.at(i).name == segmentName)
    {
      *segmentAxes = mSegments.at(i).axes;
      return true;
    }
  }
  return false;

}



Eigen::VectorXd ASFData::getSegmentDegreeOfFreedoms(std::string segmentName)
{
  if (segmentName == "root")
  {
    Eigen::Vector6d segmentDofs = Eigen::Vector6d::Ones();
    return segmentDofs;
  }
  else
  {

    Eigen::Vector3d segmentDofs = Eigen::Vector3d::Zero();
    for (int i=0; i<mSegments.size(); ++i)
    {
      if (mSegments.at(i).name == segmentName)
      {
        segmentDofs = mSegments.at(i).dofs;
      }
    }

    return segmentDofs;
  }

}

/*
bool ASFData::getSegmentLimits(std::string segmentName,
                               std::vector<std::pair<double, double>>* limits)
{
  for (int i=0; i<mSegments.size(); ++i)
  {
    if (mSegments.at(i).name == segmentName)
    {
      *limits = mSegments.at(i).limits;
      return true;
    }
  }
  return false;

}

*/


bool ASFData::getSegmentParentName(std::string segmentName,
                                   std::string * parentName)
{
  for (int i=0; i<mSegments.size(); ++i)
  {
    if (mSegments.at(i).name == segmentName)
    {
      if (mSegments.at(i).parent != nullptr)
        *parentName = mSegments.at(i).parent->name;
      else
        *parentName = "root";
      return true;
    }
  }
  return false;

}


bool ASFData::getSegmentNames(std::vector<std::string> * segmentNameList)
{
  std::vector<std::string> list(0);
  list.push_back("root");
  for (std::size_t i=0; i<mSegments.size(); ++i)
  {
    list.push_back(mSegments.at(i).name);
  }
  if (list.size() > 0)
  {
    *segmentNameList = list;
    return true;
  }
  else
    return false;
}


// =================================================================
//   subroutines for reading the data from ASF
// =================================================================
bool ASFData::readRoot()
{
  std::string line, token, dof_val;
  while (std::getline(mRetriever, line))
  {
    std::istringstream stream(line);
    stream >> token;

    if (token == ":bonedata")
      break;

    // read the order
    if (token == "order")
    {
      //stream >> root->order;
    }

    // read the axis order
    if (token == "axis")
    {
      //stream >> root->axis;
    }

    // read the direction
    if (token == "direction")
    {
      //stream >> root->direction;
    }

    // read the orientation
    if (token == "orientation")
    {
      //stream >> root->orientation;
    }


  }

  return true;
}


bool ASFData::readSegments()
{
  std::string line, token, dof_val;
  bool is_seg_begin = false;
  Segment newSegment;
  std::istringstream stream;
  double axis_buff; // axis buffer
  std::string dof_flag; // dof flag
  double direction_buff; // direction buffer
  // std::string limit_buff; // limit buffer
  // std::smatch limit_match;
  // std::regex  limit_pattern(R"(([+-]{0,1}[0-9]+\.[0-9]+))");
  // std::regex  limit_pattern("-?[0-9]+(\\.[0-9]+)?$");
  std::pair<double, double> limits; // limit pair temp

  while (std::getline(mRetriever, line))
  {
    stream.clear();
    stream.str(line);
    stream >> token;

    if (token == ":hierarchy")
    {
      break;
    } 
    // read begin -> start new segment
    if (token == "begin")
    {
      newSegment.name = "";
      newSegment.direction = Eigen::Vector3d::Zero(); // new direction
      newSegment.length = 0.0; // new direction
      newSegment.dof_flag = DOF_NONE; // make sure the segment starts with 0 DOF
      newSegment.axes = Eigen::Vector3d::Zero(); // new dofs
      newSegment.dofs = Eigen::Vector3d::Zero(); // new dofs
      newSegment.parent = nullptr; // new direction

      is_seg_begin = true;
    }

    
    while (is_seg_begin)
    {
      // input new string to stream
      stream.clear();
      std::getline(mRetriever, line);
      stream.str(line);
      
      stream >> token;

      // read end -> end old segment
      if (token == "end")
      {


        is_seg_begin = false;
        // push the new segment to bone list
        mSegments.push_back(newSegment);
        // print current data
        break;
      }

      // read id
      if (token == "id")
      {

        stream >> newSegment.id;

      }


      // read name -> replace current bone_name to read one
      if (token == "name")
      {
        stream >> newSegment.name;

      }


      // read direction
      if (token == "direction")
      {
        for (int i=0; i<3; ++i) // suppose direction is 3D
        {
          stream >> direction_buff;
          newSegment.direction(i) = direction_buff;
        }

      }


      // read length
      if (token == "length")
      {

        stream >> newSegment.length;
      }


      // read length
      if (token == "axis")
      {
        for (int i=0; i<3; ++i) // suppose direction is 3D
        {
          stream >> axis_buff;
          newSegment.axes(i) = axis_buff;
        }
      }


      // read degree of freedom
      if (token == "dof")
      {
        while(stream >> dof_flag)
        {
          if (dof_flag == "rx")
            newSegment.dof_flag += DOF_RX;
          if (dof_flag == "ry")
            newSegment.dof_flag += DOF_RY;
          if (dof_flag == "rz")
            newSegment.dof_flag += DOF_RZ;
        }

        if (newSegment.dof_flag == DOF_NONE)
        {
          newSegment.dofs = Eigen::Vector3d(0, 0, 0);
        }
        else if (newSegment.dof_flag == DOF_RX)
        {
          newSegment.dofs = Eigen::Vector3d(1, 0, 0);
        }
        else if (newSegment.dof_flag == DOF_RY)
        {
          newSegment.dofs = Eigen::Vector3d(0, 1, 0);
        }
        else if (newSegment.dof_flag == DOF_RZ)
        {
          newSegment.dofs = Eigen::Vector3d(0, 0, 1);
        }
        else if (newSegment.dof_flag == DOF_RX_RY)
        {
          newSegment.dofs = Eigen::Vector3d(1, 1, 0);
        }
        else if (newSegment.dof_flag == DOF_RX_RZ)
        {
          newSegment.dofs = Eigen::Vector3d(1, 0, 1);
        }
        else if (newSegment.dof_flag == DOF_RY_RZ)
        {
          newSegment.dofs = Eigen::Vector3d(0, 1, 1);
        }
        else if (newSegment.dof_flag == DOF_RX_RY_RZ)
        {
          newSegment.dofs = Eigen::Vector3d(1, 1, 1);
        }
        else
        {
          newSegment.dofs = Eigen::Vector3d(0, 0, 0);
        }

      } // end reading degree of freedom


      // read dof limitation
      if (token == "limits")
      {
        /*
        // first limits pair
        stream >> limit_buff;
        std::regex_match(limit_buff, limit_match, limit_pattern);
        limits.first = std::stod(limit_match[1]);

        stream >> limit_buff;
        std::regex_match(limit_buff, limit_match, limit_pattern);
        limits.second = std::stod(limit_match[1]);
        newBone.limits.push_back(limits);
*/
      }


    } // end of reading bone segment

  } // end of reading bone data

  return true;

}

bool ASFData::readHierarchy()
{

  std::istringstream stream;
  std::string line, token, parentSegmentName, childSegmentName;
  Segment * parentSegmentPtr;
  Segment * childSegmentPtr;

  // get the begin token, if not, the format is wrong
  std::getline(mRetriever, line);
  stream.clear();
  stream.str(line);
  stream >> token;
  if (token != "begin")
  {
    dtwarn << "[Wrong ASF Format]: lost begin token at hierarchy section"
           << std::endl;
    return false;
  }

  
  // attach bones
  while (std::getline(mRetriever, line))
  {
    childSegmentPtr, parentSegmentPtr = nullptr;
    stream.clear();
    stream.str(line);
    stream >> parentSegmentName;
    if (parentSegmentName == "end")
    {
      break;
    }
    else
    {
      // if parenetSegmentName == "root", pointer will be null
      parentSegmentPtr = getSegmentPtr(parentSegmentName);
    }

    while(stream >> childSegmentName)
    {
      childSegmentPtr = getSegmentPtr(childSegmentName);
      if (parentSegmentPtr != nullptr && childSegmentPtr != nullptr)
        childSegmentPtr->parent = parentSegmentPtr;

    }
  }

  //// adjust the segment reference (relate to its parent joint) based on axis
  //// data
  //JointPtr currentJointPtr;
  //JointPtr parentJointPtr;
  //BodyNodePtr currentBodyNodePtr;
  //std::string currentBodyNodeName;
  //Eigen::Vector3d rotationReference;
  //Eigen::Isometry3d transformMatrix;
  //double deg_to_rad = M_PI/180;
  //for (int i=0; i<mBones.size(); ++i)
  //{
  //  currentBodyNodeName = mBones.at(i).name;
  //  currentJointPtr = skel->getJoint(currentBodyNodeName+"_joint");
  //  currentBodyNodePtr = skel->getBodyNode(currentBodyNodeName);
  //  parentJointPtr = currentBodyNodePtr->getParentJoint();
  //
  //if (parentJointPtr->getName() != "root")
  //  {
  //    rotationReference = mBones.at(i).axes * deg_to_rad;
  //    //transformMatrix.linear() = rotationReference;
  //    //parentJointPtr->setTransform(transformMatrix);
  //    parentJointPtr->setPositions(rotationReference);
  //  }
  //}


  return true;
}


bool ASFData::readAsfData(char * asfFileUri)
{
  // init retriever
  mRetriever.open(asfFileUri, std::ios::in);
  assert(mRetriever);

  // skip macro
  std::string line, token, dof_val;
  while (std::getline(mRetriever, line))
  {
    std::istringstream stream(line);
    stream >> token;
    if (token == ":root") break;
  }


  if (!readRoot())
  {
    dtwarn << "[ASFData::readAsfData] fail reading root"
           << std::endl;
    return false;
  }

  if (!readSegments())
  {
    dtwarn << "[ASFData::readAsfData] fail reading segments"
           << std::endl;
    return false;
  }

//  // testing readSegments
//  for (int i=0; i<mSegments.size(); ++i)
//  {
//    std::cout << mSegments.at(i).dof_flag << std::endl;
//    std::cout << mSegments.at(i).dofs << std::endl;
//    std::cout << mSegments.at(i).direction << std::endl;
//  }

  
  if (!readHierarchy())
  {
    dtwarn << "[ASFData::readAsfData] fail reading Hierarchy"
           << std::endl;
    return false;
  }
  

//  // testing hierarchy
//  std::cout << getSegmentPtr("lfemur")->parent->name << std::endl;
//  int nullPtrCnt = 0;
//  for (int i=0; i<mSegments.size(); ++i)
//  {
//    if (mSegments.at(i).parent == nullptr)
//    {
//      nullPtrCnt += 1;
//    }
//  }
//  std::cout << nullPtrCnt << " nullptr" << std::endl;




  return true;

}



// ============================================================================
//  Subroutine for readSkeleton
// ============================================================================

// TODO: input Root data
bool attachRoot(dart::dynamics::SkeletonPtr skel)
{

  dart::dynamics::FreeJoint::Properties j_prop;
  j_prop.mName = "root_joint";
  dart::dynamics::BodyNode::Properties b_prop;
  b_prop.mName = "root";
  dart::dynamics::BodyNodePtr parent = nullptr;

  // create joint and body node pair
  dart::dynamics::BodyNode* bn = nullptr;

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
    nullptr, j_prop, b_prop);

  bn = pair.second;
  if (bn == nullptr)
    return false;
  else
    return true;
}


bool attachSegment(
    SkeletonPtr skel,
    std::string segmentName,
    std::string parentName)
{

  BodyNodePtr bn = nullptr;

  // create joint properties pointer
  BallJoint::Properties j_prop;
  j_prop.mName = segmentName + "_joint";


  BodyNode::Properties b_prop;
  b_prop.mName = segmentName;

  dart::dynamics::BodyNodePtr parent = skel->getBodyNode(parentName);
  auto pair = skel->createJointAndBodyNodePair<BallJoint>(
    parent, j_prop, b_prop);

  bn = pair.second;
  if (bn == nullptr)
    return false;
  else
    return true;


}





bool createKinematicTree(ASFData *asfData, dart::dynamics::SkeletonPtr skel)
{
  // attach root to skeleton
  if (!attachRoot(skel))
    dtwarn << "[AsfParser::createKinematicTree] fail adding root" << std::endl;

  // attach segments to skeleton
  std::vector<std::string> segmentNameList;
  std::string segmentName = "";
  std::string parentName = "";
  if (asfData->getSegmentNames(&segmentNameList))
  {
    for (int i=0; i<segmentNameList.size(); ++i)
    {
      segmentName = segmentNameList.at(i);
      if (segmentName != "root"
          && asfData->getSegmentParentName(segmentName, &parentName))
      {
        if (!attachSegment(skel, segmentName, parentName))
        {
          dtwarn << "[AsfParser::createKinematicTree] fail adding segment"
                 << std::endl;
          return false;
        }
      }
    }
  }


  return true;

}


// Each joint's transformation data is in parent's segment data,
// except segments which parent is root
bool modifyKinematicTree(ASFData *asfData, dart::dynamics::SkeletonPtr skel)
{
  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter
  const double deg_to_rad = M_PI/180.0;

  std::vector<std::string> segmentNameList;
  std::string segmentName = "";
  std::string parentName = "";
  Eigen::Vector3d parentAxes;
  Eigen::Vector3d segmentAxes;
  dart::dynamics::JointPtr j_segment = nullptr;
  Eigen::Isometry3d tf;
  if (asfData->getSegmentNames(&segmentNameList))
  {
    for (int i=0; i<segmentNameList.size(); ++i)
    {
      tf = Eigen::Isometry3d::Identity();
      segmentName = segmentNameList.at(i);
      if (asfData->getSegmentParentName(segmentName, &parentName))
      {
        if (parentName != "root")
        {
          asfData->getSegmentAxes(parentName, &parentAxes);
          asfData->getSegmentAxes(segmentName, &segmentAxes);
          j_segment = skel->getJoint(segmentName+"_joint");
          /*
          tf.linear() = 
              dart::math::eulerZYXToMatrix(parentAxes*deg_to_rad).transpose()
              * dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
          */
          tf.translation() = asfData->getSegmentDirection(parentName)
                             * unit
                             * asfData->getSegmentLength(parentName);
          j_segment->setTransformFromParentBodyNode(tf);
          // show the relative transform from current segment to parent
          dtmsg << "[modifyKinematicTree] " << segmentName
                << "\n-> axes in euler angle\n"
                << segmentAxes
                << "\n-> parent axes\n"
                << parentAxes
                << "\n-> segment rotation\n"
                << dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad)
                << "\n-> parent axes rotation\n"
                << dart::math::eulerZYXToMatrix(parentAxes*deg_to_rad)
                << "\n-> after: relative rotation transform\n"
                << j_segment->getTransformFromParentBodyNode().linear()
                << "\n-> after: relative translation transform\n"
                << j_segment->getTransformFromParentBodyNode().translation()
                << std::endl;
 
        }
      }

    }

  }


  return true;
}


// Attach BodyNode Shape and Joint Shape based on the data which towards to
// child.
// Root -> only joint, 0 dofs -> only body node.
bool createShapeNode(ASFData *asfData, dart::dynamics::SkeletonPtr skel)
{
  const double j_rad = 0.02; // m
  const double b_rad = 0.02; // m
  const double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter


  dart::dynamics::ShapePtr j_shape;
  dart::dynamics::ShapePtr b_shape;

  dart::dynamics::ShapeNode* j_sn;
  dart::dynamics::ShapeNode* b_sn;

 
  // Create Segment's Joint and BodyNode Shape
  
  std::vector<std::string> segmentNameList;
  std::string segmentName;
  Eigen::Isometry3d tf;
  if (asfData->getSegmentNames(&segmentNameList))
  {
    for (int i=0; i<segmentNameList.size(); ++i)
    {
      segmentName = segmentNameList.at(i);
      // if current segment has dof > 0, create joint shape
      if (segmentName == "root")
      {
        // create root's joint shape
        j_shape.reset(
            new EllipsoidShape(sqrt(2)*Eigen::Vector3d(j_rad, j_rad, j_rad))
        );
        j_sn = skel->getBodyNode(segmentName)
                   ->createShapeNodeWith<dart::dynamics::VisualAspect,
                                         dart::dynamics::CollisionAspect,
                                         dart::dynamics::DynamicsAspect>
                        (j_shape, segmentName+"_joint_shape");
        j_sn->getVisualAspect()->setColor(dart::Color::Blue());
 
      }
      else if (
          asfData->getSegmentDegreeOfFreedoms(segmentName)
          == Eigen::Vector3d::Zero()
      )
      {
        // create segment's bodynode shape only, since there's no dof on joint
        // Determine the local transform of the shape
        tf = Eigen::Isometry3d::Identity();
        tf.linear() = dart::math::computeRotation(
            asfData->getSegmentDirection(segmentName),
            dart::math::AxisType::AXIS_Z);
        tf.translation() = 0.5
                          * unit
                          * asfData->getSegmentLength(segmentName)
                          * asfData->getSegmentDirection(segmentName);                                                                  
        b_shape.reset(new CylinderShape(b_rad,
                                        asfData->getSegmentLength(segmentName)
                                        * unit));
        b_sn = skel->getBodyNode(segmentName)
            ->createShapeNodeWith<dart::dynamics::VisualAspect,   
                                  dart::dynamics::CollisionAspect,
                                  dart::dynamics::DynamicsAspect>
                                      (b_shape, segmentName+"_shape");                                                                 
        b_sn->setRelativeTransform(tf);
        b_sn->getVisualAspect()->setColor(dart::Color::Black());  
      }
        
      else
      {
        // create joint shape
        j_shape.reset(
            new EllipsoidShape(sqrt(2)*Eigen::Vector3d(j_rad, j_rad, j_rad))
        );

        j_sn = skel->getBodyNode(segmentName)
                   ->createShapeNodeWith<dart::dynamics::VisualAspect,
                                         dart::dynamics::CollisionAspect,
                                         dart::dynamics::DynamicsAspect>
                        (j_shape, segmentName+"_joint_shape");
        j_sn->getVisualAspect()->setColor(dart::Color::Blue());

        // create segment's segment bodynode shape 
        tf = Eigen::Isometry3d::Identity();
        tf.linear() = dart::math::computeRotation(
            asfData->getSegmentDirection(segmentName),
            dart::math::AxisType::AXIS_Z);
        tf.translation() = 0.5
                          * unit
                          * asfData->getSegmentLength(segmentName)
                          * asfData->getSegmentDirection(segmentName);                                                                  
        b_shape.reset(new CylinderShape(b_rad,
                                        asfData->getSegmentLength(segmentName)
                                        * unit));
        b_sn = skel->getBodyNode(segmentName)
            ->createShapeNodeWith<dart::dynamics::VisualAspect,   
                                  dart::dynamics::CollisionAspect,
                                  dart::dynamics::DynamicsAspect>
                                      (b_shape, segmentName+"_shape");                                                                 
        b_sn->setRelativeTransform(tf);
        b_sn->getVisualAspect()->setColor(dart::Color::Black());  

      }
//      std::cout << segmentName << std::endl;
//      std::cout << asfData->getSegmentDegreeOfFreedoms(segmentName) << std::endl;
//      std::cout << asfData->getSegmentDirection(segmentName) << std::endl;
//      std::cout << skel->getBodyNode(segmentName)->getRelativeTransform().translation()
//                << std::endl;

    }
  }



}



dart::dynamics::SkeletonPtr readSkeleton(char *asfFileUri,
                                         char *skelName,
                                         ASFData **asfData)
{

  dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create(skelName);


  // TODO: put asfData outside
  *asfData = new ASFData();
  std::vector<std::string> segmentNames;
  if (!(*asfData)->getSegmentNames(&segmentNames))
  {
    dtwarn << "[readSkeleton]: init new asfData, error in getting segmentNames"
           << std::endl;
  }
  else
  {
    dtmsg << "[readSkeleton]: init new segmentNames size = "
          << segmentNames.size()
          << std::endl;
  }

  if (!(*asfData)->readAsfData(asfFileUri))
  {
    dtwarn << "[readSkeleton]: Unable to read ASF Data, return null skeleton"
           << std::endl;
    return nullptr;
  }

  if (!(*asfData)->getSegmentNames(&segmentNames))
  {
    dtwarn << "[readSkeleton]: after read asfData, error in getting segmentNames"
           << std::endl;
  }
  else
  {
    dtmsg << "[readSkeleton]: after read asfData, segmentNames size = "
          << segmentNames.size()
          << std::endl;
  }


  if (!createKinematicTree(*asfData, skel))
  {
    dtwarn << "[readSkeleton]: Unable to create kinematic tree based on data, "
           << "return nullptr."
           << std::endl;
    return nullptr;

  }

  // testing kinematic tree
  std::cout << skel->getNumBodyNodes() << " BodyNodes" << std::endl;


  modifyKinematicTree(*asfData, skel);

  // testing modified kinematic tree
  //std::cout << skel->getJoint("lfingers_joint")->getTreeIndex()
  //          << std::endl;



  createShapeNode(*asfData, skel);
  // test createShapeNode position
  std::cout << "lfinger's bodynode frame world coord" << std::endl;
  std::cout << skel->getBodyNode("lfingers")->getTransform().translation()
            << std::endl;

  return skel;
}
