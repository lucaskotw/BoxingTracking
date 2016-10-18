/**
 * The parsing process debunk as follows:
 *   1) read the Root and Segment data
 *   2) read hierachy structure
 * 
 * After parsing, one can utilize the readSkeleton method to
 *   1) create Joint and BodyNode pair (parent: Joint, child: BodyNode)
 *   2) transform joint based on hierarchical strucure to generate a kinematic
 *      tree.
 *   3) create the ShapeNode attaching to the BodyNodes
 *
 * TODO:
 *   1) read the limit data
 *   2) consider root's order and axis data
 *
 */
#ifndef BOXINGTRACKING_ASFPARSER_H_
#define BOXINGTRACKING_ASFPARSER_H_

#include <dart/dart.hpp>
#include <vector>
#include <utility> // pair
#include <regex> // extract pair from limit
#include <string>
#include <fstream>
#include <sstream>


// Constants for dof_flag
const int DOF_NONE     = 0;
const int DOF_RX       = 1;
const int DOF_RY       = 2;
const int DOF_RZ       = 4;
const int DOF_RX_RY    = 3;
const int DOF_RX_RZ    = 5;
const int DOF_RY_RZ    = 6;
const int DOF_RX_RY_RZ = 7;


struct Root
{
  //char * order;
  //char * axis;
  std::vector<double> position;
  std::vector<double> orientation;
};


struct Segment
{
  int id;
  std::string name;
  Eigen::Vector3d direction;
  double length;
  int dof_flag;
  Eigen::Vector3d axes;
  Eigen::Vector3d dofs;
  // std::vector<std::pair<double, double>> limits;
  Segment * parent;

  // Constructor
  //Segment(const int& id = 0,
  //        const std::string& name = "",
  //        const Eigen::Vector3d& direction = Eigen::Vector3d::Zero(),
  //        double length = 0.0,
  //        int dof_flag = 0,
  //        const Eigen::Vector3d& axes = Eigen::Vector3d::Zero(),
  //        const Eigen::Vector3d& dofs = Eigen::Vector3d::Zero(),
  //        // limit init
  //        Segment * parent = nullptr
  //       );
       

};


class ASFData
{
private:
  // members
  std::ifstream mRetriever;
  Root mRoot;
  std::vector<Segment> mSegments;

  // getter for Root and Segment, which are supposed to be accessed only by
  // instance itself
  Segment * getSegmentPtr(std::string segmentName);


  // subroutines for reading the data from ASF
  bool readSegments();
  bool readRoot();
  bool readHierarchy();


public:
  // constructor, copy constructor and destructor
  ASFData();
  //ASFData(const ASFData & asfData);
  //ASFData& operator= (const ASFData & asfData);
  ~ASFData();


  // read asfdata
  bool readAsfData(char * asfFileUri);

  // getter and setter
  Eigen::Vector3d getSegmentDirection(std::string segmentName);
  double getSegmentLength(std::string segmentName);
  bool getSegmentAxes(std::string segmentName,
                      Eigen::Vector3d * segmentAxes);
  Eigen::VectorXd getSegmentDegreeOfFreedoms(std::string segmentName);
  bool getSegmentLimits(std::string segmentName,
                        std::vector<std::pair<double, double>>* limits);

  bool getSegmentParentName(std::string segmentName,
                                     std::string * parentName);

  bool getSegmentNames(std::vector<std::string> * segmentNameList);
  
  // load data from reading the ASF file

};

dart::dynamics::SkeletonPtr readSkeleton(char * asfFileUri);

#endif
