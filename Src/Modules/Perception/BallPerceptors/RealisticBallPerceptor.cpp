#include "RealisticBallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Transformation.h"
#include "Platform/SystemCall.h"
#include <algorithm>
#include "Representations/Infrastructure/Image.h"
#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace cv;


MAKE_MODULE(RealisticBallPerceptor, perception)

void RealisticBallPerceptor::update(RealisticBallPercepts& ballPercepts)
{
    if(che==false){
    	string configDirectory;
		char currentWorkingDirectory[1024];

		configDirectory = "";

		if (SystemCall::getMode() == SystemCall::simulatedRobot)
		{
			if (getcwd(currentWorkingDirectory,1024)) {;}

			configDirectory = currentWorkingDirectory;
			configDirectory = configDirectory.substr(0,configDirectory.rfind("/")) + "/";
			configDirectory = configDirectory + "Cascade/";
		}
		else configDirectory = "Config/Cascade/";
		ball_upper_cascade = configDirectory + "cascade.xml";
		ball_lower_cascade = configDirectory + "cascade.xml";

		upper_cascade.load(ball_upper_cascade);
		lower_cascade.load(ball_lower_cascade);
		che=true;
		    
    }


    
    const float minDistanceFromLineSquared = sqr(minDistanceFromLine);
    const float centerCircleRadiusSquared = sqr(theFieldDimensions.centerCircleRadius);
    

    if(LowerBall == false || theCameraInfo.camera == CameraInfo::Camera::lower)
    {
        LowerBall = false;
        bool find = false;
        
        ballPercepts.balls.clear();
        for(const Boundaryi& region : theBallRegions.regions)
        {
        	int RegionL;
        	int RegionR;
        	int RegionT;
        	int RegionB;
        	if(theCameraInfo.camera == CameraInfo::Camera::upper)
        	{
		        RegionL = std::max((int(-region.x.getSize()*magnification_upper+region.x.min+region.x.max)/2), 0);
		        RegionR = std::min((int(region.x.getSize()*magnification_upper+region.x.min+region.x.max)/2), theCameraInfo.width-1);
		        RegionT = std::max((int(-region.y.getSize()*magnification_upper+region.y.min+region.y.max)/2), 0);
		        RegionB = std::min((int(region.y.getSize()*magnification_upper+region.y.min+region.y.max)/2), theCameraInfo.height-1);
            }
            else
        	{
		        RegionL = std::max((int(-region.x.getSize()*magnification_lower+region.x.min+region.x.max)/2), 0);
		        RegionR = std::min((int(region.x.getSize()*magnification_lower+region.x.min+region.x.max)/2), theCameraInfo.width-1);
		        RegionT = std::max((int(-region.y.getSize()*magnification_lower+region.y.min+region.y.max)/2), 0);
		        RegionB = std::min((int(region.y.getSize()*magnification_lower+region.y.min+region.y.max)/2), theCameraInfo.height-1);
            }
            int BlackCount = 0;
             LINE("module:RealisticBallPerceptor:square", RegionL , RegionT, RegionR , RegionT, 1 , Drawings::solidPen, ColorRGBA::blue);
                    LINE("module:RealisticBallPerceptor:square", RegionL , RegionT, RegionL , RegionB, 1 , Drawings::solidPen, ColorRGBA::blue);
                    LINE("module:RealisticBallPerceptor:square", RegionR , RegionT, RegionR , RegionB, 1 , Drawings::solidPen, ColorRGBA::blue);
                    LINE("module:RealisticBallPerceptor:square", RegionL, RegionB, RegionR , RegionB, 1 , Drawings::solidPen, ColorRGBA::blue);
             
            Mat src_img(RegionB-RegionT, RegionR-RegionL, CV_8UC3);
            Mat src_BGR;
            
            for(unsigned short x = (unsigned short) RegionL; x < RegionR; x++)
            {
                for(unsigned short y = (unsigned short) RegionT; y < RegionB; y++)
                {
                    if(theECImage.colored[y][x]==FieldColors::black)
                        BlackCount ++;
                        
                    unsigned char Y, Cr, Cb;
                    Image::Pixel current = theImage.getFullSizePixel(y, x);
                    Y=(unsigned char)current.y;
                    Cr=(unsigned char)current.cr;
                    Cb=(unsigned char)current.cb;
                    src_img.at<Vec3b>(y-RegionT,x-RegionL)=Vec3b(Y,Cr,Cb);
                }
            }
            

	        vector<Rect>BALL;
            cvtColor(src_img, src_BGR, CV_YCrCb2BGR);
            if(theCameraInfo.camera == CameraInfo::Camera::upper)
            	upper_cascade.detectMultiScale(src_BGR, BALL);
            else
            	lower_cascade.detectMultiScale(src_BGR, BALL);
            vector<cv::Rect>::const_iterator iter = BALL.begin();
            RealisticBallPercept ball;
            while(iter!=BALL.end())
            {
            	LINE("module:RealisticBallPerceptor:cascade", RegionL + iter->x , RegionT + iter->y, RegionL + iter->x + iter->width , RegionT + iter->y, 2 , Drawings::solidPen, ColorRGBA::red);
                LINE("module:RealisticBallPerceptor:cascade", RegionL + iter->x , RegionT + iter->y, RegionL + iter->x , RegionT + iter->y + iter->height, 2 , Drawings::solidPen, ColorRGBA::red);
                LINE("module:RealisticBallPerceptor:cascade", RegionL + iter->x + iter->width , RegionT + iter->y, RegionL + iter->x + iter->width , RegionT + iter->y + iter->height, 2 , Drawings::solidPen, ColorRGBA::red);
                LINE("module:RealisticBallPerceptor:cascade", RegionL + iter->x, RegionT + iter->y + iter->height, RegionL + iter->x + iter->width , RegionT + iter->y + iter->height, 2 , Drawings::solidPen, ColorRGBA::red);
                
                int CenterX = RegionL + iter->x + iter->width/2;
                int CenterY = RegionT + iter->y + iter->height/2;
                
                if(theCameraInfo.camera == CameraInfo::Camera::lower)
                    LowerBall = true;
                ball.positionInImage.x() = CenterX;
                ball.positionInImage.y() = CenterY;
                ball.radiusInImage = iter->height - iter->width > 0 ? iter->width/2: iter->height/2;
                
                if(calculateBallOnField(ball)){
                    validatePerceptPreField(ball);
                    validatePerceptPostField(ball, minDistanceFromLineSquared, centerCircleRadiusSquared);
                    
                    ballPercepts.balls.push_back(ball);
                    find = 1;
                }
                ++iter;
            }
        }
    }
    else ballPercepts.balls.clear();
}



bool RealisticBallPerceptor::calculateBallOnField(RealisticBallPercept& ballPercept) const
{
    Vector2f leftPosition = theImageCoordinateSystem.toCorrected(Vector2f(ballPercept.positionInImage.x() - ballPercept.radiusInImage, ballPercept.positionInImage.y()));
    Vector2f rightPosition = theImageCoordinateSystem.toCorrected(Vector2f(ballPercept.positionInImage.x() + ballPercept.radiusInImage, ballPercept.positionInImage.y()));
    if(Transformation::imageToRobotHorizontalPlane(leftPosition, expectedRadius, theCameraMatrix, theCameraInfo, leftPosition) &&
       Transformation::imageToRobotHorizontalPlane(rightPosition, expectedRadius, theCameraMatrix, theCameraInfo, rightPosition))
    {
        ballPercept.radiusOnField = (leftPosition - rightPosition).norm() / 2.0f;
        if(Transformation::imageToRobotHorizontalPlane(theImageCoordinateSystem.toCorrected(ballPercept.positionInImage),
                                                       ballPercept.radiusOnField, theCameraMatrix, theCameraInfo, ballPercept.relativePositionOnField))
        {
            ballPercept.absolutePositionOnField = theRobotPose * ballPercept.relativePositionOnField;
            return !validateFieldRadius ||
            (ballPercept.radiusOnField >= minRadiusOnField &&
             ballPercept.radiusOnField <= maxRadiusOnField);
        }
    }
    
    return false;
}





bool RealisticBallPerceptor::validatePerceptPostField(const RealisticBallPercept& ball, float minDistanceFromLineSquared, float centerCircleRadiusSquared) const
{
    LinesPercept::Line l;
    return theFieldDimensions.isInsideField(ball.absolutePositionOnField) &&
    ball.relativePositionOnField.x() >= minDistance &&
    (theECImage.colored[static_cast<int>(ball.positionInImage.y())][static_cast<int>(ball.positionInImage.x())]!=FieldColors::white ||
     std::abs(theFieldDimensions.fieldLinesWidth - ball.radiusOnField * 2) >= minDiffFromLineWidth ||
     ((ball.absolutePositionOnField - theFieldDimensions.fieldLinesWithGoalFrame.getClosestPoint(ball.absolutePositionOnField)).squaredNorm() >= minDistanceFromLineSquared &&
      std::abs(ball.absolutePositionOnField.squaredNorm() - centerCircleRadiusSquared) >= minDistanceFromLineSquared &&
      std::abs(ball.absolutePositionOnField.squaredNorm() - centerCircleRadiusSquared) >= minDistanceFromLineSquared &&
      (std::abs(ball.absolutePositionOnField.y()) >= minDistanceFromPenaltyMark ||
       std::abs(ball.absolutePositionOnField.x() - theFieldDimensions.xPosOpponentPenaltyMark) >= minDistanceFromPenaltyMark)));
}

bool RealisticBallPerceptor::validatePerceptPreField(const RealisticBallPercept& ball) const
{
    const int centerX = static_cast<int>(ball.positionInImage.x());
    const int centerY = static_cast<int>(ball.positionInImage.y());
    
    if(centerX >= 0 &&
       centerX < theCameraInfo.width &&
       centerY > (theFieldBoundary.isValid ? std::max(0, theFieldBoundary.getBoundaryY(centerX)) : 0) &&
       centerY < theCameraInfo.height)
    {
        if(theCameraInfo.camera == CameraInfo::Camera::lower)
        {
            int clippedBottom = centerY;
            theBodyContour.clipBottom(centerX, clippedBottom, theCameraInfo.height);
            if(centerY > clippedBottom)
            {
                return false;
            }
        }
        
        
        
        const int r = static_cast<int>(ball.radiusInImage);
        const int rDiagonal = r * 7 / 10; // approximately sin(45deg) = cos(45deg)
        int greenCounter = 0;
        int checked = 1;
        if(theECImage.colored[centerY][centerX]==FieldColors::green)
        {
            greenCounter++;
        }
        for(int x = centerX - 1, y = centerY - 1; x >= std::max(0, centerX - rDiagonal) && y >= std::max(0, centerY - rDiagonal); x--, y--)
        {
            checked++;
            if(theECImage.colored[y][x]==FieldColors::green)
            {
                greenCounter++;
            }
        }
        for(int x = centerX + 1, y = centerY - 1; x < std::min(theCameraInfo.width, centerX + 1 + rDiagonal) && y >= std::max(0, centerY - rDiagonal); x++, y--)
        {
            checked++;
            if((theECImage.colored[y][x]==FieldColors::green))
            {
                greenCounter++;
            }
        }
        for(int x = centerX + 1, y = centerY + 1; x < std::min(theCameraInfo.width, centerX + 1 + rDiagonal) && y < std::min(theCameraInfo.height, centerY + 1 + rDiagonal); x++, y++)
        {
            checked++;
            if((theECImage.colored[y][x]==FieldColors::green))
            {
                greenCounter++;
            }
        }
        for(int x = centerX - 1, y = centerY + 1; x >= std::max(0, centerX - rDiagonal) && y < std::min(theCameraInfo.height, centerY + 1 + rDiagonal); x--, y++)
        {
            checked++;
            if(theECImage.colored[y][x]==FieldColors::green)
            {
                greenCounter++;
            }
        }
        for(int x = centerX, y = centerY - 1; y >= std::max(0, centerY - r); y--)
        {
            checked++;
            if(theECImage.colored[y][x]==FieldColors::green)
            {
                greenCounter++;
            }
        }
        for(int x = centerX, y = centerY + 1; y < std::min(theCameraInfo.height, centerY + 1 + r); y++)
        {
            checked++;
            if(theECImage.colored[y][x]==FieldColors::green)
            {
                greenCounter++;
            }
        }
        for(int x = centerX - 1, y = centerY; x >= std::max(0, centerX - r); x--)
        {
            checked++;
            if(theECImage.colored[y][x]==FieldColors::green)
            {
                greenCounter++;
            }
        }
        for(int x = centerX + 1, y = centerY; x < std::min(theCameraInfo.width, centerX + 1 + r); x++)
        {
            checked++;
            if(theECImage.colored[y][x]==FieldColors::green)
            {
                greenCounter++;
            }
        }
        
        const float greenPart = (float)greenCounter / (float)checked;
        return greenPart < maxGreenThreshold || greenPart > minGreenThreshold;
    }
    
    return false;
}



