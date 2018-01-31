//ボールを見る．

option(LookBall, (float) (0.38f) tilt)
{
  /** Simply sets the necessary angles */
  initial_state(lookBall)
  {
    action
    {
      SetHeadPanTilt(theBallModel.estimate.position.angle(), -0.1f, 150_deg);
    }
  }
}
