//広範囲に首を左右に動かす動作（Readyの歩行前に行う）

option(LookAroundWide, (float) (0.38f) tilt)
{
  /** Simply sets the necessary angles */
  initial_state(lookAroundWide)
  {
    transition
    {
      goto lookLeft;
    }
    action
    {
      SetHeadPanTilt(0.f, -0.1f, 150_deg);
    }
  }

  state(lookLeft)
  {
    transition
    {
      if(state_time > 2500)
        goto lookRight;
    }
    action
    {
      SetHeadPanTilt(1.0f, -0.1f, 50_deg);
    }
  }

  state(lookRight)
  {
    transition
    {
      if(state_time > 2500)
        goto lookLeft;
    }
    action
    {
      SetHeadPanTilt(-1.0f, -0.1f, 50_deg);
    }
  }
}
