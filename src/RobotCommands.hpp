/*
 * headingChange_node.hpp
 *
 *  Created on: Feb 26, 2016
 *      Author: Michael
 */

#ifndef SRC_ROBOTCOMMANDS_HPP_
#define SRC_ROBOTCOMMANDS_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/CameraInfo.h>
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/ArmTrajectoryRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include "ihmc_msgs/ChestTrajectoryRosMessage.h"
#include "ihmc_msgs/PelvisHeightTrajectoryRosMessage.h"
#include "ihmc_msgs/WholeBodyTrajectoryRosMessage.h"
#include "ihmc_msgs/StopAllTrajectoryRosMessage.h"
#include "ihmc_msgs/WalkingStatusRosMessage.h"
#include "ihmc_msgs/HighLevelStateChangeStatusRosMessage.h"
#include "ihmc_msgs/HandTrajectoryRosMessage.h"
#include "ihmc_msgs/HeadTrajectoryRosMessage.h"
//#include <opencv2/opencv.hpp>

class RobotCommands
{
public:
	RobotCommands();
	~RobotCommands();

	void generateFootstepList(double);
	void generateFootstepList();
	void generateStairClimbList(int);
	void publishStepList();
	void generateArmMotion(bool,int);
	ihmc_msgs::ChestTrajectoryRosMessage generateTorsoMove(bool);
	void generatePelvisMove(double);
	int getNumStepsTaken();
	void centerFeet();
	void closeFeet();

	ihmc_msgs::FootstepDataListRosMessage getFootStepList();

	double swingTime;
	double transferTime;
	bool stillWalking;

private:

	tf::TransformListener tfListener;

	ihmc_msgs::FootstepDataRosMessage createFootStep(bool, tf::Transform);
	ihmc_msgs::FootstepDataRosMessage createStairStep(bool, tf::Vector3);
	ihmc_msgs::ArmTrajectoryRosMessage createArmMotion(bool,int);
	ihmc_msgs::HandTrajectoryRosMessage createCartesianArmMotion();
	ihmc_msgs::FootTrajectoryRosMessage createFootMotion(bool);
	geometry_msgs::Quaternion convertQuat(tf::Quaternion);
	geometry_msgs::Vector3 convertVec(tf::Vector3);
	ihmc_msgs::ChestTrajectoryRosMessage createTorsoLean(bool);
	ihmc_msgs::HeadTrajectoryRosMessage createHeadLean(bool);
	ihmc_msgs::HeadTrajectoryRosMessage createHeadRotate(bool);

	ihmc_msgs::FootstepDataListRosMessage footStepList;
	ihmc_msgs::FootstepDataListRosMessage stairClimbList;

	int numStepsTaken;

	//cv::Mat lastImage;
	//cv::Mat currentImage;
	//tf::StampedTransform cameraOpticTransform;
	//std::vector<cv::Point> walkPath;
	//std::vector<cv::Point> handlePoints;
	//std::vector<tf::Vector3> handleXYZ;

//	double fx;
//	double fy;
//	double Tx;
//	double Ty;
//	double cx;
//	double cy;
	bool feetInitialized;

	tf::Vector3 getGroundIntersect3D(int, int);
	tf::Vector3 getHandleIntersect3D(int, int);
	void addStraightSteps(double);
	void addSideSteps(double);
	void addHeadingChangeSteps(double);

	void makeHeadingZero();
	void resetStepList();
	void updateFootFrames();

	tf::Transform lastRightFootFrame;
	tf::Transform lastLeftFootFrame;
};

void fillArray(double,double,double,double,double,double,double,double*);

#endif /* SRC_ROBOTCOMMANDS_HPP_ */
