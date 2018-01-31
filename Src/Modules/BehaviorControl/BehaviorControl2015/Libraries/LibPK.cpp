#include "../LibraryBase.h"

namespace Behavior2015
{
	#include "LibPK.h"

	LibPK::LibPK()
	{
	InMapFile stream("BehaviorControl2015/libPK.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
	}


	void LibPK::preProcess(){
	angleToGoalRight = (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGroundline, -350.f)).angle(); 

	angleToGoalLeft = (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGroundline, 370.f)).angle(); 
	}
	


	
	
	
	bool LibPK::leftrightrobot()
    {
        int righttrue=0;
        int center = 10000;
	for (auto itr = theObstacleModel.obstacles.begin();itr != theObstacleModel.obstacles.end(); itr++)
		{
			Vector2f robot=Transformation::robotToField(theRobotPose.translation,Vector2f(itr->center.x(),itr->center.y()));
            if(std::abs(center-righttrue)>std::abs(robot.y()-righttrue))
                center=int(robot.y());
        }

        
         for(const PlayersPercept::Player& it : thePlayersPercept.players)
      {
      
      Vector2f robot=Transformation::robotToField(theRobotPose.translation,Vector2f(it.centerOnField.x(),it.centerOnField.y()));
            if(std::abs(center-righttrue)>std::abs(robot.y()-righttrue))
                center=int(robot.y());
        }


        if(center > righttrue)return true;else return false;

    }
	
	
}
