/* Keeper Motion when PK */
option(PkGoalie)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 500) 
        goto walk;
    }
    action
    {
      LookForward();
      Stand();
    }
  }

  state(walk)
  {
    transition
    {
       	if(state_time > 3500)
       		goto waitAround;
    }
    action
    {
    	WalkToTarget(Pose2f(0.f,0.5f,0.f),Pose2f(0.f,1000.f,0.f));
      LookBall();
    }
  }

  state(wait)
  {
  	transition
  	{
  		if(libCodeRelease.timeSinceBallWasSeen() >2000)
  			goto waitAround;
		  if(state_time > 3000)
			   goto BGsub;
  	}
  	action
  	{
  	   LookStop();
  	   Stand();
  	}
  }

  state(waitAround)
  {
  	transition
  	{
    		if(libCodeRelease.timeSinceBallWasSeen() < 2000)
  			   goto wait;
  	}
	  action
  	{
        LookAroundWide();
  			Stand();
  	}
	}


  state(BGsub)
  {
  	transition
  	{
  		if(theBallPercept.BSball)
  			goto falldown;
  	}
  	action
	  {
		LookStop();
        Stand();
	   }
  }


  state(falldown)
  {
    transition
    {
    	if(action_done && state_time > 13500)
    		goto stand;
    }
    action
    {
      SpecialAction(SpecialActionRequest::preventBall);
    }
  }


  state(stand)
  {
    transition
    {
    	if(state_time > 300)
    		goto waitAround;
    }
    action
    {
    	LookStop();
      Stand();
    }
  }
}
