option(PkStriker)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 500)
         goto gotoball;
    }
    action
    {
      LookForward();
      Stand();

    }
  }
  state(gotoball) 
  {
    transition 
    {
	if(libCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 100.f) && libCodeRelease.between(theBallModel.estimate.position.x(), 250.f, 350.f))
	goto gotoballnear;
    }
    action 
    {
	LookDown();
	WalkToTarget(Pose2f(45_deg, 0.5f, 0.8f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x()-300.f, theBallModel.estimate.position.y()-30.f));
    }
  }
 state(gotoballnear) 
  {
    transition 
    {
	if(libCodeRelease.between(theBallModel.estimate.position.y(), 0.f, 60.f) && libCodeRelease.between(theBallModel.estimate.position.x(), 200.f, 350.f))
	goto wait;
    }
    action 
    {
	LookDown();
  WalkToTarget(Pose2f(45_deg, 0.3f, 0.8f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 270.f, theBallModel.estimate.position.y() - 30.f));
    }
  }
  state(wait) 
  {
    transition
    {
  if(state_time > 2000 && libPK.leftrightrobot())
	goto rightwalk;
	if(state_time > 2000 && !libPK.leftrightrobot())
	goto straightwalk;
    }
    action
    {
    Stand();
	LookForward();
    }
  }
  state(rightwalk)
  {
    transition
    {
	if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) && libCodeRelease.between(theBallModel.estimate.position.x(), 50.f, 200.f) && std::abs(libPK.angleToGoalRight) < 2_deg)
	goto pk;
    }
    action
    {
    LookDown();
	WalkToTarget(Pose2f(45_deg, 0.15f, 0.5f), Pose2f(libPK.angleToGoalRight, theBallModel.estimate.position.x()-160.f, theBallModel.estimate.position.y()-30.f));
    }
  }
  state(straightwalk) 
  {
    transition
    {
	if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) && libCodeRelease.between(theBallModel.estimate.position.x(), 150.f, 200.f) && std::abs(libPK.angleToGoalLeft) < 2_deg)
	goto pk;
    }
    action
    {
    LookDown();
	WalkToTarget(Pose2f(45_deg, 0.15f, 0.5f), Pose2f(libPK.angleToGoalLeft, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 30.f));
}
  }
  state(pk)
  {
	transition
    {
  		if(state_time > 5000 && action_done)
          goto gotoball;
    }
    action
    {
    	LookDown();
        InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }
  }
}
