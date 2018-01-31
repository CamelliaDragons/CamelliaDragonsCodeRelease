option(LookStop)
{
  /** Simply sets the necessary angles */
  initial_state(lookStop)
  {
    action
    {
      SetHeadPanTilt(0.f, -0.1f, 150_deg);
    }
  }
}
