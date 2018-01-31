/**
 * @file RealisticBallPerceptor.h
 * @author Felix Thielke
 */

#pragma once

#include <cmath>
#include <set>
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/BallPercepts/RealisticBallPercepts.h"
#include "Representations/Perception/BallPercepts/BallRegions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Approx.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#define MAX_WINDOW_RADIUS 64

MODULE(RealisticBallPerceptor,
{,
  REQUIRES(BallRegions),
  REQUIRES(ECImage),
  REQUIRES(BodyContour),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(LinesPercept),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(CameraInfo),
  USES(RobotPose),
  REQUIRES(RealisticBallPercepts),
  PROVIDES(RealisticBallPercepts),
  DEFINES_PARAMETERS(
  {
    ,
    (bool)(true) validatePre,
    (bool)(true) validateFieldRadius,
    (bool)(true) validatePost,
    (float)(0.1f) maxGreenThreshold,
    (float)(1) minGreenThreshold,
    (float)(35) expectedRadius,
    (float)(120) minDistance,
    (float)(32) minRadiusOnField,
    (float)(220) maxRadiusOnField,
    (float)(50) minDiffFromLineWidth,
    (float)(200) minDistanceFromLine,
    (float)(200) minDistanceFromPenaltyMark,
  }),
});

class RealisticBallPerceptor : public RealisticBallPerceptorBase
{
cv::CascadeClassifier upper_cascade;
cv::CascadeClassifier lower_cascade;
std::string ball_upper_cascade;
std::string ball_lower_cascade;



float magnification_upper = 2.0f;
float magnification_lower = 2.0f;
bool che = false;
bool LowerBall = false;

private:

  void update(RealisticBallPercepts& ballPercepts);


  void cleanup();

 
  bool validatePerceptPreField(const RealisticBallPercept& ballPercept) const;
  bool calculateBallOnField(RealisticBallPercept& ballPercept) const;
  bool validatePerceptPostField(const RealisticBallPercept& ballPercept, float minDistanceFromLineSquared, float centerCircleRadiusSquared) const;


};
