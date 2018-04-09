#include "PriorityProvider.h"
#include <string>
#include <cmath>
#include <math.h>
#include <iomanip>

#include <string>

MAKE_MODULE(PriorityProvider, behaviorControl)

  void PriorityProvider::update(Priority& priority)
  {
	  priority.getoneRank = getoneRank();
	  priority.getPRank = getPRank();
	  priority.isBestAgent = isBestAgent();
	  priority.supportPose = calcSupportPose();

  }


  int PriorityProvider::timeSinceBallWasSeen()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }

  bool PriorityProvider::between(float value, float min, float max)
  {
    return value >= min && value <= max;
  }

void PriorityProvider::calcPriority(){
	float x, y, d, theta, dis, dir;
	x = theBallModel.estimate.position.x();
	y = theBallModel.estimate.position.y();
	theta = (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();

	x /= 500.f;
	y /= 500.f;
	d = (float)sqrt(pow(x, 2)+pow(y, 2));

	if(d < 1.f){
		d = 1.0f;
	}
	dir = (float)((cos(theta)+1.0)/2.0);
	dis = 1.0f/d;
	priority[theRobotInfo.number] = (alpha*dir + beta*dis)/(alpha+beta);


	for(auto const& teammate : theTeammateData.teammates){

		if( teammate.status == Teammate::PLAYING || timeSinceBallWasSeen() > 3000 ){

			x = teammate.ball.estimate.position.x();
			y = teammate.ball.estimate.position.y();
			theta = (teammate.pose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();

			x /= 500.f;
			y /= 500.f;
			d = (float)sqrt(pow(x, 2)+pow(y, 2));

			if(d < 1.f){
				d = 1.0f;
			}
			dir = (float)((cos(theta)+1.0)/2.0);
			dis = 1.0f/d;
			priority[teammate.number] = (alpha*dir + beta*dis)/(alpha+beta);
		}else{
			priority[teammate.number] = 0.f;
		}
	}
	checkActivePlayers();
	priority[theRobotInfo.number] = calcMyPriority();
}


float PriorityProvider::calcMyPriority(){
	float x, y, d, theta, dis, dir;
	int myNumber;

	if( timeSinceBallWasSeen() < 3000.f ){
		myNumber = theRobotInfo.number;
		x = theBallModel.estimate.position.x();
		y = theBallModel.estimate.position.y();
		x /= 500.f;			//Normalize
		y /= 500.f;
		d = (float)sqrt(pow(x, 2)+pow(y, 2));
		if(d < 1.f){
			d = 1.0f;
		}
		theta = angleToGoal;
		dir = (float)((cos(theta)+1.0)/2.0);
		dis = 1.0f/d;
		return (alpha*dir + beta*dis)/(alpha+beta);
	}
	return 0.f;
}


bool PriorityProvider::isBestAgent(){
	float buffer=0.f;
	int number=0;
	for(int i=1; i<6; i++){
		if(buffer<priority[i]){
			buffer=priority[i];
			number=i;
		}
	}
	if(theRobotInfo.number == number){
		return true;
	}
	return false;
}


int PriorityProvider::getoneRank(){
	float buffer2=0.f;
	int oneRank=0;
	for(int i=1; i<=5; i++){
		if(buffer2<priority[i]){
			buffer2=priority[i];
			oneRank=i;
		}
	}
	return oneRank;
}


int PriorityProvider::getPRank(){
	int pRank=1;
	float myPriority;

	myPriority=priority[theRobotInfo.number];
	for(int i=1; i<=5; i++){
		if(i==theRobotInfo.number){
		}else if(priority[i] > myPriority){
			pRank++;
		}
	}
	return pRank;
}


Vector2f PriorityProvider::calcSupportPose(){
	int stRank = getoneRank();

	float dis = 3000.f;

	if(theRobotInfo.number == stRank){
		if(0.7 < theRobotPose.validity){
			distiX = (float)(theRobotPose.translation.x() + cos(theRobotPose.rotation)*(dis));
			distiY = (float)(theRobotPose.translation.y() + sin(theRobotPose.rotation)*(dis));
		}
		else{
			distiX = (float)(theBallModel.estimate.position.x() + dis);
			distiY = (float)(theBallModel.estimate.position.y());
		}
		if(distiX > 4000.f) distiX = 4000.f;
		if(distiY > 2500.f) distiY = 2500.f;

		return (Vector2f(distiX,distiY));
	}


	for(auto const& teammate : theTeammateData.teammates){
		if(teammate.number == stRank){
			if(0.7 < teammate.pose.validity){
				distiX = (float)(teammate.pose.translation.x() + cos(teammate.pose.rotation)*(dis));
				distiY = (float)(teammate.pose.translation.y() + sin(teammate.pose.rotation)*(dis));

			}
			else{
				distiX = (float)(theBallModel.estimate.position.x() + dis);
				distiY = (float)(theBallModel.estimate.position.y());
			}

		}
	}
	if(distiX > 4000.f) distiX = 4000.f;
	if(distiY > 2500.f) distiY = 2500.f;

	return (Vector2f(distiX,distiY));

}

void PriorityProvider::checkActivePlayers(){

	for(int i=0; i<=6; i++){
		if(check[i] >= 10)
			priority[i] = 0.f;
	}
	for(auto const& teammate : theTeammateData.teammates){
		if(teammate.timeWhenLastPacketReceived <= timeLastReceived[teammate.number]){
			check[teammate.number]++;
		}else{
			check[teammate.number] = 0;
			timeLastReceived[teammate.number] = teammate.timeWhenLastPacketReceived;
		}
	}
}
