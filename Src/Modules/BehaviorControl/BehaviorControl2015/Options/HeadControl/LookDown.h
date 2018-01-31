//足元を見る．
//多分これで下向きめいいっぱい
//  by kzh 2017-06-23
option(LookDown)
{
  initial_state(lookDown)
  {
    action
    {
      //SetHeadPanTilt(0.f, 0.0f, 150_deg);
      SetHeadPanTilt(0.f, 0.38f, 150_deg);
    }
  }
}
