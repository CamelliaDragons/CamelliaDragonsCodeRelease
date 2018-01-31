class LibPK : public LibraryBase
{
public:
  STREAMABLE(Parameters,
  {,
    (bool) isPKActive,
    (bool) isGoalieActive,
  });
  Parameters parameters;                       
  LibPK();

	void preProcess() override;
	
	float angleToGoalRight;
    float angleToGoalLeft;
    bool leftrightrobot();



	Vector2f gloBallPos;
	float calcDistance(Vector2f start, Vector2f end);
	float angleRltvRobot(Vector2f posAbsl);
	Vector2f rltvCoordToAbslCoord(Vector2f rltvCood);
	Vector2f abslCoordToRltvCoord(Vector2f abslCoord);
	
};
