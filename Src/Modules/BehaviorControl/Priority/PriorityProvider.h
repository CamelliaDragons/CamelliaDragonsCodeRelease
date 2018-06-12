


#pragma once
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Communication/TeammateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Priority.h"
#include "Tools/Module/Module.h"



MODULE(PriorityProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(TeammateData),
  REQUIRES(BallModel),
  REQUIRES(RobotPose),
  PROVIDES(Priority),
});

class PriorityProvider : public PriorityProviderBase
{

  float alpha=1.0f, beta=10.0f;
  int count=0;
  void calcPriority();
  float calcPriority(int agentNumber);
  float calcMyPriority();

  void checkActivePlayers();

  void update(Priority& priority);

public:

  bool between(float value, float min, float max);

  int timeSinceBallWasSeen();

  float priority[6];

  bool isBestAgent();

  int getoneRank();

  int getPRank();

  float distiX, distiY;

  Vector2f calcSupportPose();

  Vector2f translation;

  float angleToGoal;

  int check[6] = {0, 0, 0, 0, 0, 0};
  unsigned timeLastReceived[6] = {0, 0, 0, 0, 0, 0};


};
