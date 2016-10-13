/**
 * Author: Lucas Ko <lucaskointw@gmail.com>
 */
#ifndef MOTIONINTERPOLATION_MYWINDOW_H_
#define MOTIONINTERPOLATION_MYWINDOW_H_

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <iostream> // use to display the reading result
#include <fstream>  // load data
#include <string>   // for line buffer
#include "AsfParser.h"

class MyWindow : public dart::gui::SimWindow
{
public:
  // Constructor
  //MyWindow(dart::simulation::WorldPtr world);
  MyWindow() : SimWindow() {}

  // ~MyWindow();
  virtual ~MyWindow() {}

  void setSkel(dart::dynamics::SkeletonPtr _skel);
  void timeStepping() override;

private:
  ASFData * mAsfData;
  int mTimeStepCnt;
  dart::dynamics::SkeletonPtr mSkel;


};


#endif