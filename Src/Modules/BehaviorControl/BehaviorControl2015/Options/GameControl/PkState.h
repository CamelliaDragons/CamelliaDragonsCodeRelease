option(PkState)
{
  initial_state(PK)
  {
    transition
    {

    }
    action
    {
      if(libPK.parameters.isGoalieActive)
      	PkGoalie();
      else
      	PkStriker();
    }
  }
}

