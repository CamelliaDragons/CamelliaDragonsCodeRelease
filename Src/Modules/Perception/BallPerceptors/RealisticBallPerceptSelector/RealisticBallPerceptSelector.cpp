/**
* @file RealisticBallPerceptSelector.cpp
* @author Felix Thielke
*/

#include <algorithm>
#include <cfloat>
#include "Tools/Math/Approx.h"
#include "RealisticBallPerceptSelector.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(RealisticBallPerceptSelector, modeling)

RealisticBallPerceptSelector::RealisticBallPerceptSelector()
{
}

void RealisticBallPerceptSelector::updateRobots()
{
  robots.clear();
  for(const Obstacle& o : theObstacleModel.obstacles)
  {
    robots.emplace_back(o.center.squaredNorm(), Angle::normalize(Approx::atan2(o.right.y(), o.right.x()) - robotSideAngleDisplacement), Angle::normalize(Approx::atan2(o.left.y(), o.left.x()) + robotSideAngleDisplacement));
  }
}

void RealisticBallPerceptSelector::update(BallPercept& ballPercept)
{
   DECLARE_DEBUG_DRAWING("module:RealisticBallPerceptor:notseen", "drawingOnImage");

  updateRobots();

  ballPercept.status = BallPercept::notSeen;
  //const float maxDistanceFromLastModelSquared = maxDistanceFromLastModel * maxDistanceFromLastModel;
  const float robotFrontDisplacementSquared = robotFrontDisplacement * robotFrontDisplacement;
  float minDistance = FLT_MAX;
  for(const RealisticBallPercept& percept : theRealisticBallPercepts.balls)
  {

  	if(percept.BSball)
	{
		const float distance = percept.relativePositionOnField.squaredNorm();
	    minDistance = distance;
        ballPercept.positionInImage = percept.positionInImage;
        ballPercept.radiusInImage = percept.radiusInImage;
        ballPercept.positionOnField = percept.relativePositionOnField;
        ballPercept.radiusOnField = percept.radiusOnField;
        ballPercept.status = BallPercept::seen;
        ballPercept.BSball = true;
        goto nextPercept;

	}

    if(theFieldDimensions.fieldBorder.isInside(percept.absolutePositionOnField))
    {
      const float ballStart = Angle::normalize(Approx::atan2(percept.relativePositionOnField.y() + percept.radiusOnField, percept.relativePositionOnField.x()));
      const float ballEnd = Angle::normalize(Approx::atan2(percept.relativePositionOnField.y() - percept.radiusOnField, percept.relativePositionOnField.x()));
      for(const PlayersPercept::Player& p : thePlayersPercept.players)
      {
        if(!p.lowerCamera && p.x1 <= percept.positionInImage.x() + percept.radiusInImage &&
           p.x2 >= percept.positionInImage.x() - percept.radiusInImage &&
           p.y1 <= percept.positionInImage.y() + percept.radiusInImage &&
           p.y2 >= percept.positionInImage.y() - percept.radiusInImage)
        {
	      DRAWTEXT("module:RealisticBallPerceptor:notseen",percept.positionInImage.x(), percept.positionInImage.y(), 7, ColorRGBA::red, "InImage");
          goto nextPercept;


        }
      }
      for(const RobotData& r : robots)
      {
        if(percept.relativePositionOnField.squaredNorm() >= r.distance - robotFrontDisplacementSquared && ballEnd >= r.start && ballStart <= r.end)
        {
	        DRAWTEXT("module:RealisticBallPerceptor:notseen",percept.positionInImage.x(), percept.positionInImage.y(), 7, ColorRGBA::red, "RobotDis");
          goto nextPercept;
        }
      }
    }

    if(1)
    {
      const float distance = percept.relativePositionOnField.squaredNorm();
      if(distance < minDistance)
      {
        minDistance = distance;
        ballPercept.positionInImage = percept.positionInImage;
        ballPercept.radiusInImage = percept.radiusInImage;
        ballPercept.positionOnField = percept.relativePositionOnField;
        ballPercept.radiusOnField = percept.radiusOnField;
        ballPercept.status = BallPercept::seen;
      }
      else
        DRAWTEXT("module:RealisticBallPerceptor:notseen",percept.positionInImage.x(), percept.positionInImage.y(), 7, ColorRGBA::red, "lastingnow");
    }

  nextPercept:
    ;
  }
}
