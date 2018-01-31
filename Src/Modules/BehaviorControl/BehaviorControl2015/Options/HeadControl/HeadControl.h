option(HeadControl)
{
  common_transition
  {
    if(!theGroundContactState.contact && theGameInfo.state != STATE_INITIAL)
      goto lookForward;

    switch(theHeadControlMode)
    {
      case HeadControl::off: goto off;
      case HeadControl::lookForward: goto lookForward;
      //case HeadControl::lookForwardUp: goto lookForwardUp;
      //case HeadControl::lookAround: goto lookAround;
      case HeadControl::lookAroundWide: goto lookAroundWide;
      case HeadControl::lookBall: goto lookBall;
	  case HeadControl::lookStop: goto lookStop;
      //case HeadControl::lookDown: goto lookDown;
      //case HeadControl::lookingBall: goto lookingBall;
      //case HeadControl::lookForBall: goto lookForBall;
      default: goto none;
    }
  }

  initial_state(none) {}
  state(off) {action SetHeadPanTilt(JointAngles::off, JointAngles::off, 0.f);}
  state(lookForward)    {action LookForward();}
/*  state(lookForwardUp)  {action LookForwardUp();}
  state(lookAround)	{action LookAround();}*/
  state(lookAroundWide)	{action LookAroundWide();}
  state(lookBall)	{action LookBall();}
  state(lookStop)	{action LookStop();}
 /* state(lookDown)	{action LookDown();}
  state(lookingBall)	{action LookingBall();}
  state(lookForBall)	{action LookForBall();}*/
}

struct HeadControl
{
  ENUM(Mode,
  {,
    none,
    off,
    lookForward,
/*    lookForwardUp,
    lookAround,*/
    lookAroundWide,
    lookBall,
	lookStop,
/*    lookDown,
    lookingBall,
    lookForBall,*/
  });
};

HeadControl::Mode theHeadControlMode = HeadControl::Mode::none; /**< The head control mode executed by the option HeadControl. */
