#pragma once

#include "Tools/Math/Eigen.h"


STREAMABLE(Priority,
{,
  (Vector2f)(Vector2f::Zero()) supportPose,
  (bool)(false) isBestAgent,
  (int)(0) getoneRank,
  (int)(0) getPRank,
});
