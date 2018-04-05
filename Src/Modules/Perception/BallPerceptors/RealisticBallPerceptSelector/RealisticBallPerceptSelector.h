/**
 * @file RealisticBallPerceptSelector.h
 * @author Felix Thielke
 */

#pragma once

#include <unordered_set>
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/PlayersPercepts/PlayersPercept.h"
#include "Representations/Perception/BallPercepts/RealisticBallPercepts.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/OutStreams.h"

MODULE(RealisticBallPerceptSelector,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(PlayersPercept),
  REQUIRES(RealisticBallPercepts),
  USES(ObstacleModel),
  USES(BallModel),
  PROVIDES(BallPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(1000) robotFrontDisplacement,
    (Angle)(5_deg) robotSideAngleDisplacement,
    (int)(1000) timeUntilForget,
    (float)(300.f) maxDistanceFromLastModel,
  }),
});

/**
 * @class RealisticBallPerceptSelector
 */
class RealisticBallPerceptSelector : public RealisticBallPerceptSelectorBase
{
public:
  RealisticBallPerceptSelector();

private:
  struct RobotData
  {
  public:
    const float distance;
    const Angle start;
    const Angle end;
    RobotData(const float distance, const Angle start, const Angle end) : distance(distance), start(start), end(end) {}
  };
  std::vector<RobotData> robots;

  void update(BallPercept& ballPercept);

  void updateRobots();
};
