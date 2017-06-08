//#include <ros/ros.h>
//#include <std_msgs/Float64.h>
//#include <tf/transform_listener.h>
//#include <std_msgs/Empty.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>

#include "RobotCommands.hpp"

RobotCommands::RobotCommands()
{
	ROS_INFO("in class constructor of RobotCommands");

	swingTime = 0.5;
	transferTime = 0.2;

	feetInitialized = false;
	numStepsTaken = 0;
	stillWalking = false;
}

RobotCommands::~RobotCommands()
{
	//cv::destroyWindow("left camera view");
}

void fillArray(double a,double b,double c,double d,double e,double f,double g, double* output)
{
	output[0] = a;
	output[1] = b;
	output[2] = c;
	output[3] = d;
	output[4] = e;
	output[5] = f;
	output[6] = g;
}


void RobotCommands::resetStepList()
{
	if (footStepList.footstep_data_list.size() > 0)
	{
		footStepList.footstep_data_list.clear();
	}

	footStepList.default_swing_time = swingTime;
	footStepList.default_transfer_time = transferTime;
	footStepList.execution_mode = 0 ; //3 = obstacle avoidance
	footStepList.unique_id = 2;
}


void RobotCommands::centerFeet()
{

	tf::StampedTransform rightFootFrameInPelvis;
	tf::StampedTransform leftFootFrameInPelvis;
	tf::StampedTransform pelvisFrame;

	try{
		tfListener.lookupTransform("pelvis", "rightFoot", ros::Time(0), rightFootFrameInPelvis);
		tfListener.lookupTransform("pelvis", "leftFoot", ros::Time(0), leftFootFrameInPelvis);
		tfListener.lookupTransform("world", "pelvis", ros::Time(0), pelvisFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	rightFootFrameInPelvis.getOrigin().setValue(-0.032,-0.15,rightFootFrameInPelvis.getOrigin().getZ());
	leftFootFrameInPelvis.getOrigin().setValue(-0.032,0.15,leftFootFrameInPelvis.getOrigin().getZ());

	//tf::Vector3 test = rightFootFrameInPelvis.getOrigin();
	//tf::Vector3 test2 = pelvisFrame*rightFootFrameInPelvis
	//ROS_INFO("Right X = %f, Y = %f, Z = %f",test.getX(), test.getY(), test.getZ());

	footStepList.footstep_data_list.push_back(createFootStep(true, pelvisFrame*rightFootFrameInPelvis));
	footStepList.footstep_data_list.push_back(createFootStep(false, pelvisFrame*leftFootFrameInPelvis));

	numStepsTaken = 0;
	feetInitialized = true;
}

void RobotCommands::closeFeet()
{

	tf::StampedTransform rightFootFrameInPelvis;
	tf::StampedTransform leftFootFrameInPelvis;
	tf::StampedTransform pelvisFrame;

	try{
		tfListener.lookupTransform("pelvis", "rightFoot", ros::Time(0), rightFootFrameInPelvis);
		tfListener.lookupTransform("pelvis", "leftFoot", ros::Time(0), leftFootFrameInPelvis);
		tfListener.lookupTransform("world", "pelvis", ros::Time(0), pelvisFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	rightFootFrameInPelvis.getOrigin().setValue(-0.032,-0.09,rightFootFrameInPelvis.getOrigin().getZ());
	leftFootFrameInPelvis.getOrigin().setValue(-0.032,0.09,leftFootFrameInPelvis.getOrigin().getZ());

	//tf::Vector3 test = rightFootFrameInPelvis.getOrigin();
	//tf::Vector3 test2 = pelvisFrame*rightFootFrameInPelvis
	//ROS_INFO("Right X = %f, Y = %f, Z = %f",test.getX(), test.getY(), test.getZ());

	footStepList.footstep_data_list.push_back(createFootStep(true, pelvisFrame*rightFootFrameInPelvis));
	footStepList.footstep_data_list.push_back(createFootStep(false, pelvisFrame*leftFootFrameInPelvis));

	numStepsTaken = 0;
}

// Creates footstep with the current position and orientation of the foot.
ihmc_msgs::FootstepDataRosMessage RobotCommands::createFootStep(bool isRight, tf::Transform footFrame)
{
	ihmc_msgs::FootstepDataRosMessage footstep;
	std::string footFrameName;
	if (isRight)
	{
		footstep.robot_side = 1;
		lastRightFootFrame = footFrame;
	}
	else
	{
		footstep.robot_side = 0;
		lastLeftFootFrame = footFrame;
	}

	footstep.orientation = this->convertQuat(footFrame.getRotation());
	footstep.location = this->convertVec(footFrame.getOrigin());

	footstep.swing_height = 0.05;

	//ROS_INFO("isRight = %d, X = %f, Y = %f, Z = %f",isRight, footstep.location.x,
	//		footstep.location.y, footstep.location.z);

	return footstep;
}

geometry_msgs::Quaternion RobotCommands::convertQuat(tf::Quaternion input)
{
	geometry_msgs::Quaternion output;
	output.w = input.getW();
	output.x = input.getX();
	output.y = input.getY();
	output.z = input.getZ();
	return output;
}

geometry_msgs::Vector3 RobotCommands::convertVec(tf::Vector3 input)
{
	geometry_msgs::Vector3 output;
	output.x = input.getX();
	output.y = input.getY();
	output.z = input.getZ();
	return output;
}



ihmc_msgs::HandTrajectoryRosMessage RobotCommands::createCartesianArmMotion()
{
	//ihmc_msgs::ArmTrajectoryRosMessage armMotion;

	ihmc_msgs::HandTrajectoryRosMessage handMotion;

	handMotion.unique_id = 2;
	handMotion.base_for_control = handMotion.WORLD;
	handMotion.robot_side = handMotion.RIGHT;
	handMotion.execution_mode = handMotion.OVERRIDE;

	ihmc_msgs::SE3TrajectoryPointRosMessage handTraj;

	handTraj.time = 1.0;
	//handTraj.position.x = 4.0;
	//handTraj.position.y = 0.68;
	//handTraj.position.z = 1.0;

//	handTraj.position.x = 0.05;
//	handTraj.position.y = -0.7;
//	handTraj.position.z = 1.1; //rpy all 0

//	handTraj.position.x = 0.45;
//	handTraj.position.y = -0.4;
//	handTraj.position.z = 1.1; //yaw  = pi/2

	//handTraj.position.x = (handleXYZ[0].getX() + handleXYZ[1].getX())/2 - 0.05;
	//handTraj.position.y = (handleXYZ[0].getY() + handleXYZ[1].getY())/2 - 0.05;
	//handTraj.position.z = (handleXYZ[0].getZ() + handleXYZ[1].getZ())/2;
	//handTraj.position.x = 3.78;
	//handTraj.position.y = 0.715;
	//handTraj.position.z = 0.98;

	handTraj.position.x = 0.19;
	handTraj.position.y = -0.632;
	handTraj.position.z = 0.666;

	tf::Quaternion myTest;
	//myTest.setRPY(1.572, 0.0, 1.348);
	myTest.setRPY(1.396, -0.003, 0.569);

	handTraj.orientation = this->convertQuat(myTest);
	handTraj.linear_velocity = geometry_msgs::Vector3();
	handTraj.angular_velocity = geometry_msgs::Vector3();

	handMotion.taskspace_trajectory_points.push_back(handTraj);

	return handMotion;

}

ihmc_msgs::ArmTrajectoryRosMessage RobotCommands::createArmMotion(bool doRight, int posNum)
{
	ihmc_msgs::ArmTrajectoryRosMessage armMotion;

	armMotion.robot_side = armMotion.RIGHT;
	if (doRight == false)
		armMotion.robot_side = armMotion.LEFT;

	armMotion.unique_id = 1;

	ihmc_msgs::OneDoFJointTrajectoryRosMessage holder;
	for (int i = 0; i < 7; i++)
	{
		armMotion.joint_trajectory_messages.push_back(ihmc_msgs::OneDoFJointTrajectoryRosMessage());
	}

	ihmc_msgs::TrajectoryPoint1DRosMessage point1;
	//ihmc_msgs::TrajectoryPoint1DRosMessage point2;

	//double positionsRight[] = {-0.4, 0.6, 0.7, 1.5, 1.3, 0.0, 0.0};
	double positionsRight[7];
	if (doRight && posNum == 0)
		fillArray(-0.4, 0.6, 0.7, 1.5, 1.3, 0.0, 0.0,positionsRight);
	else if (doRight && posNum == 1)
		fillArray(-0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,positionsRight);
	else if (doRight && posNum == 2)
		fillArray(-0.4, 0.0,1.57, 0.97, 0.0, 0.0, 0.0,positionsRight);
	else if (doRight && posNum == 3)
		fillArray(-0.4, 0.6, 1.57, 0.97, 0.0, 0.0, 0.0,positionsRight);
	else if (doRight && posNum == 4)
		fillArray(-0.4, 0.6, 1.57, 0.97, -1.0, 0.0, 0.0,positionsRight);


	double positionsLeft[] = {-0.4, -0.6, 0.7, -1.5, 1.3, 0.0, 0.0};

	for (int i = 0; i < 7; i++)
	{

		if (doRight)
		{
			point1.position = positionsRight[i];
			point1.time = 0.5;
		}
		else
		{
			point1.position = positionsLeft[i];
			point1.time = 0.5;
		}
		point1.velocity = 0;
		armMotion.joint_trajectory_messages[i].trajectory_points.push_back(point1);

//		if (doRaise)
//		{
//			point2.position = positions2[i];
//			point2.time = 1.0;
//		}
//		else
//		{
//			point2.position = positions1[i];
//			point2.time = 0.5;
//		}
//
//		point2.velocity = 0;
//		armMotion.joint_trajectory_messages[i].trajectory_points.push_back(point2);

		//point3.time = 4.0;
		//point3.position = positions1[i];
		//point3.velocity = 0;
		//armMotion.joint_trajectory_messages[i].trajectory_points.push_back(point3);
	}

	return armMotion;
}

ihmc_msgs::ChestTrajectoryRosMessage RobotCommands::createTorsoLean(bool isForward)
{
	ihmc_msgs::ChestTrajectoryRosMessage leanMotion;

	ihmc_msgs::SO3TrajectoryPointRosMessage goalPoint;

	tf::StampedTransform chestFrame;

	try{
		tfListener.lookupTransform("world", "torso", ros::Time(0), chestFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	double pitchRad = 0.4;
	if (isForward == false)
		pitchRad = -0.4;

	tf::Quaternion deltaOrientation = tf::createQuaternionFromRPY(0.0, pitchRad, 0);

	goalPoint.orientation = this->convertQuat(chestFrame.getRotation()*deltaOrientation);
	goalPoint.time = 1.0;
	goalPoint.angular_velocity = this->convertVec(tf::Vector3(0,0,0));
	goalPoint.unique_id = 1;

	leanMotion.taskspace_trajectory_points.push_back(goalPoint);

	return leanMotion;
}

ihmc_msgs::HeadTrajectoryRosMessage RobotCommands::createHeadLean(bool isDown)
{
	ihmc_msgs::HeadTrajectoryRosMessage leanMotion;

	leanMotion.unique_id = 1;

	ihmc_msgs::SO3TrajectoryPointRosMessage goalPoint;

	tf::StampedTransform headFrame;

	try{
		tfListener.lookupTransform("world", "head", ros::Time(0), headFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::Quaternion deltaOrientation;

	if (isDown == true)
		deltaOrientation = tf::createQuaternionFromRPY(0.0, 0.4, 0);
	else
		deltaOrientation = tf::createQuaternionFromRPY(0.0, 0.0, 0);

	goalPoint.orientation = this->convertQuat(headFrame.getRotation()*deltaOrientation);
	//goalPoint.orientation = this->convertQuat(deltaOrientation);
	goalPoint.time = 1.0;
	goalPoint.angular_velocity = this->convertVec(tf::Vector3(0,0,0));
	goalPoint.unique_id = 1;

	leanMotion.taskspace_trajectory_points.push_back(goalPoint);

	return leanMotion;
}

ihmc_msgs::HeadTrajectoryRosMessage RobotCommands::createHeadRotate(bool isRight)
{
	ihmc_msgs::HeadTrajectoryRosMessage rotateMotion;

	rotateMotion.unique_id = 1;

	ihmc_msgs::SO3TrajectoryPointRosMessage goalPoint;

	tf::StampedTransform headFrame;

	try{
		tfListener.lookupTransform("world", "head", ros::Time(0), headFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::Quaternion deltaOrientation;

	if (isRight == true)
		deltaOrientation = tf::createQuaternionFromRPY(0.0, 0.0, 0.4);
	else
		deltaOrientation = tf::createQuaternionFromRPY(0.0, 0.0, -0.4);

	//goalPoint.orientation = this->convertQuat(deltaOrientation*headFrame.getRotation());
	//goalPoint.orientation = this->convertQuat(headFrame.getRotation()*deltaOrientation);

	goalPoint.orientation = this->convertQuat(deltaOrientation);
	goalPoint.time = 1.0;
	goalPoint.angular_velocity = this->convertVec(tf::Vector3(0,0,0));
	goalPoint.unique_id = 1;

	rotateMotion.taskspace_trajectory_points.push_back(goalPoint);

	return rotateMotion;
}

ihmc_msgs::FootTrajectoryRosMessage RobotCommands::createFootMotion(bool isRight)
{
	ihmc_msgs::FootTrajectoryRosMessage footMotion;

	std::string footFrameName;

	if (isRight)
	{
		footMotion.robot_side = footMotion.RIGHT;
		footFrameName = "rightFoot";
	}
	else
	{
		footMotion.robot_side = footMotion.LEFT;
		footFrameName = "leftFoot";
	}

	footMotion.unique_id = 1;

	ihmc_msgs::SE3TrajectoryPointRosMessage point1;
	ihmc_msgs::SE3TrajectoryPointRosMessage point2;

	tf::StampedTransform footFrame;

	try{
		tfListener.lookupTransform("world", footFrameName.c_str(), ros::Time(0), footFrame);
		//tfListener.lookupTransform(footFrameName.c_str(), "world", ros::Time(0), footFrame);
		//ROS_INFO("Got both Transforms!");
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	point1.orientation = this->convertQuat(footFrame.getRotation());
	point1.position = this->convertVec(footFrame.getOrigin());

	point2.orientation = this->convertQuat(footFrame.getRotation());
	point2.position = this->convertVec(footFrame.getOrigin());
	//footstep.trajectory_type = 2; //Push recovery (says it is for fast steps)

	tf::Vector3 offset1 = tf::Vector3(0.15, 0, 0.1);
	tf::Vector3 offset2 = tf::Vector3(0.3, 0, 0);

	tf::Vector3 newLocation1 = footFrame*offset1;
	tf::Vector3 newLocation2 = footFrame*offset2;

	point1.position = this->convertVec(newLocation1);
	point2.position = this->convertVec(newLocation2);

	point1.time = 0.3;
	point2.time = 0.6;

	footMotion.taskspace_trajectory_points.push_back(point1);
	footMotion.taskspace_trajectory_points.push_back(point2);

	footMotion.unique_id = 1;


	return footMotion;
}

void RobotCommands::generateArmMotion(bool doRight, int posNum)
{
	ihmc_msgs::ArmTrajectoryRosMessage armMotion = this->createArmMotion(doRight,posNum);
	//int numSubscribers = armTrajPub.getNumSubscribers();
	//ROS_INFO("Number of Subscribers to Arm Publisher= %d", numSubscribers);
	//armTrajPub.publish(armMotion);

	//ihmc_msgs::HandTrajectoryRosMessage handMotion = this->createCartesianArmMotion(doRaise);
	//int numSubscribers = handTrajPub.getNumSubscribers();
	//ROS_INFO("Number of Subscribers to Hand Trajectory Publisher= %d", numSubscribers);
	//armMotion.execution_mode = armMotion.OVERRIDE;
	//handTrajPub.publish(handMotion);

}

void RobotCommands::generateFootstepList(double deltaHeadingDeg)
{
	double maxIncrementDeg = 5;

	double numStepsNeeded = ceil(fabs(deltaHeadingDeg)/maxIncrementDeg);
	double stepIncrement = deltaHeadingDeg/numStepsNeeded;

	footStepList.default_swing_time = swingTime;
	footStepList.default_transfer_time = transferTime;
	footStepList.execution_mode = 0 ; //3 = obstacle avoidance
	footStepList.unique_id = 2;

	if (footStepList.footstep_data_list.size() > 0)
	{
		footStepList.footstep_data_list.clear();
	}

	tf::StampedTransform rightFootFrame;
	tf::StampedTransform leftFootFrame;
	tf::StampedTransform pelvisFrame;

	try{
		tfListener.lookupTransform("world", "rightFoot", ros::Time(0), rightFootFrame);
		tfListener.lookupTransform("world", "leftFoot", ros::Time(0), leftFootFrame);
		tfListener.lookupTransform("world", "pelvis", ros::Time(0), pelvisFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::Vector3 averageFootFrame = (rightFootFrame.getOrigin() + leftFootFrame.getOrigin())/2;

	tf::Transform pivotPoint;
	pivotPoint.setOrigin(averageFootFrame);
	//pivotPoint.setRotation(rightFootFrame.getRotation());
	pivotPoint.setRotation(pelvisFrame.getRotation());

	ROS_INFO("Pelvis Rotation = %f, %f, %f, %f",pelvisFrame.getRotation().getW(),
			pelvisFrame.getRotation().getX(),
			pelvisFrame.getRotation().getY(),
			pelvisFrame.getRotation().getZ());

	tf::Vector3 deltaRight = rightFootFrame.getOrigin() - pivotPoint.getOrigin();
	tf::Vector3 deltaLeft = leftFootFrame.getOrigin() - pivotPoint.getOrigin();

	//pre multiply for world, post multiply for local

	tf::Transform pivotPointRotated = pivotPoint;

	tf::Transform newRightFoot;
	tf::Transform newLeftFoot;

	for (int i = 0; i < numStepsNeeded; i++)
	{
		pivotPointRotated.setRotation(pivotPoint.getRotation()*tf::createQuaternionFromRPY(0, 0, stepIncrement*(i+1)));
		newRightFoot = pivotPointRotated;
		newLeftFoot = pivotPointRotated;
		newRightFoot.setOrigin(newRightFoot*deltaRight);
		newLeftFoot.setOrigin(newLeftFoot*deltaLeft);
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(true, newRightFoot));
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(false, newLeftFoot));
	}
	numStepsTaken = 0;
}

void RobotCommands::updateFootFrames()
{
	tf::StampedTransform rightFootFrame;
	tf::StampedTransform leftFootFrame;

	try{
		tfListener.lookupTransform("world", "rightFoot", ros::Time(0), rightFootFrame);
		tfListener.lookupTransform("world", "leftFoot", ros::Time(0), leftFootFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	lastRightFootFrame = rightFootFrame;
	lastLeftFootFrame = leftFootFrame;

	feetInitialized = true;
}

void RobotCommands::addStraightSteps(double dist)
{
	double maxStepSize = 0.2;
	double numStepsNeeded = ceil(fabs(dist)/maxStepSize);
	double actualStepSize = dist/numStepsNeeded;
	tf::Vector3 stepOffset;

	if (feetInitialized == false)
		updateFootFrames();

	tf::Transform rightFootInit = lastRightFootFrame;
	tf::Transform leftFootInit = lastLeftFootFrame;

	tf::Transform newRightFoot;// = rightFootInit;
	tf::Transform newLeftFoot;// = leftFootInit;
	tf::Transform stepFrame;

	for (int i = 0; i < numStepsNeeded; i++)
	{
		stepFrame.setIdentity();
		stepOffset = tf::Vector3(actualStepSize*(i+1), 0, 0);
		stepFrame.setOrigin(stepOffset);
		if (i%2 == 0)
		{
			newRightFoot = rightFootInit*stepFrame;
			footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(true, newRightFoot));
		}
		else
		{
			newLeftFoot = leftFootInit*stepFrame;
			footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(false, newLeftFoot));
		}
	}
	stepOffset = tf::Vector3(dist, 0, 0);
	stepFrame.setIdentity();
	stepFrame.setOrigin(stepOffset);
	if (int(numStepsNeeded) % 2 == 0)
	{
		newRightFoot = rightFootInit*stepFrame;
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(true, newRightFoot));
	}
	else
	{
		newLeftFoot = leftFootInit*stepFrame;
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(false, newLeftFoot));
	}
	ROS_INFO("Added %f steps for straight portion", numStepsNeeded+1);
}

void RobotCommands::addSideSteps(double dist)
{
	double maxStepSize = 0.2;

	if (feetInitialized == false)
		updateFootFrames();

	if (fabs(dist) > maxStepSize)
		dist = (dist / fabs(dist)) * maxStepSize;

	double actualStepSize = dist;
	tf::Vector3 stepOffset;

	tf::Transform rightFootInit = lastRightFootFrame;
	tf::Transform leftFootInit = lastLeftFootFrame;

	tf::Transform newRightFoot;// = rightFootInit;
	tf::Transform newLeftFoot;// = leftFootInit;
	tf::Transform stepFrame;


	stepFrame.setIdentity();

	ROS_INFO("Distance of sidestep is %f",dist);

	stepOffset = tf::Vector3(0, actualStepSize, 0);
	stepFrame.setOrigin(stepOffset);
	if (dist > 0)
	{
		newLeftFoot = leftFootInit*stepFrame;
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(false, newLeftFoot));
		newRightFoot = rightFootInit*stepFrame;
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(true, newRightFoot));
	}
	else
	{
		newRightFoot = rightFootInit*stepFrame;
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(true, newRightFoot));
		newLeftFoot = leftFootInit*stepFrame;
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(false, newLeftFoot));
	}
}

void RobotCommands::addHeadingChangeSteps(double deltaHeadingDeg)
{
	ROS_INFO("Requested heading change of %f degrees", deltaHeadingDeg);

	double maxIncrementDeg = 20;

	double numStepsNeeded = ceil(fabs(deltaHeadingDeg)/maxIncrementDeg);
	double stepIncrementDeg = (deltaHeadingDeg/numStepsNeeded);
	ROS_INFO("Heading increment = %f degrees", stepIncrementDeg);

	tf::Transform rightFootInit = lastRightFootFrame;
	tf::Transform leftFootInit = lastLeftFootFrame;

	tf::Vector3 averageFootFrame = (rightFootInit.getOrigin() + leftFootInit.getOrigin())/2;

	tf::Transform pivotPoint;
	pivotPoint.setOrigin(averageFootFrame);
	pivotPoint.setRotation(rightFootInit.getRotation());
	//pivotPoint.setRotation(pelvisFrame.getRotation());

	tf::Vector3 deltaRight = rightFootInit.getOrigin() - pivotPoint.getOrigin();
	tf::Vector3 deltaLeft = leftFootInit.getOrigin() - pivotPoint.getOrigin();

	//pre multiply for world, post multiply for local

	tf::Transform pivotPointRotated = pivotPoint;

	tf::Transform newRightFoot;
	tf::Transform newLeftFoot;

	for (int i = 0; i < numStepsNeeded; i++)
	{
		pivotPointRotated.setRotation(pivotPoint.getRotation()*tf::createQuaternionFromRPY(0, 0, stepIncrementDeg*(i+1)/57.29578));
		newRightFoot = pivotPointRotated;
		newLeftFoot = pivotPointRotated;
		newRightFoot.setOrigin(newRightFoot*deltaRight);
		newLeftFoot.setOrigin(newLeftFoot*deltaLeft);
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(true, newRightFoot));
		footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(false, newLeftFoot));
	}
	ROS_INFO("Added %f steps for heading change", numStepsNeeded*2);

	tf::Vector3 averageFootOrigin = (lastRightFootFrame.getOrigin() + lastLeftFootFrame.getOrigin())/2;
	newRightFoot.setOrigin(averageFootFrame);
	newLeftFoot.setOrigin(averageFootOrigin);
	tf::Transform rightOffset;
	tf::Transform leftOffset;
	rightOffset.setIdentity();
	rightOffset.setOrigin(tf::Vector3(0,-0.15,0));
	leftOffset.setIdentity();
	leftOffset.setOrigin(tf::Vector3(0,0.15,0));
	footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(true, newRightFoot*rightOffset));
	footStepList.footstep_data_list.push_back(RobotCommands::createFootStep(false, newLeftFoot*leftOffset));
}

void RobotCommands::publishStepList()
{
//	footstepListPub.publish(footStepList);
}


int RobotCommands::getNumStepsTaken()
{
	return numStepsTaken;
}


ihmc_msgs::ChestTrajectoryRosMessage RobotCommands::generateTorsoMove(bool isForward)
{
	ihmc_msgs::ChestTrajectoryRosMessage chestMotion = this->createTorsoLean(isForward);

	chestMotion.previous_message_id = 0;
	chestMotion.unique_id = 1;

	return chestMotion;
}

void RobotCommands::generatePelvisMove(double deltaHeight)
{
	tf::StampedTransform pelvisFrame;

	try{
		tfListener.lookupTransform("world", "pelvis", ros::Time(0), pelvisFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	ihmc_msgs::PelvisHeightTrajectoryRosMessage pelvisMotion;
	pelvisMotion.unique_id = 1;

	ihmc_msgs::TrajectoryPoint1DRosMessage deltaPelvis;
	deltaPelvis.position = pelvisFrame.getOrigin().getZ() + deltaHeight;
	deltaPelvis.time = 1.0;
	deltaPelvis.velocity = 0.0;

	pelvisMotion.trajectory_points.push_back(deltaPelvis);
	//pelvisHeightPub.publish(pelvisMotion);
}

void RobotCommands::makeHeadingZero()
{
	if (footStepList.footstep_data_list.size() > 0)
	{
		footStepList.footstep_data_list.clear();

		footStepList.default_swing_time = swingTime;
		footStepList.default_transfer_time = transferTime;
		footStepList.execution_mode = 0 ; //3 = obstacle avoidance
		footStepList.unique_id = 2;
	}

	centerFeet();

	tf::StampedTransform pelvisFrame;
	try{
		tfListener.lookupTransform("world", "pelvis", ros::Time(0), pelvisFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::Matrix3x3 pelvisRotation = tf::Matrix3x3(pelvisFrame.getRotation());
	double curHeadingDeg = atan2(pelvisRotation.getColumn(0).getY(),
			pelvisRotation.getColumn(0).getX())*57.29578;

	addHeadingChangeSteps(-curHeadingDeg);
	publishStepList();
}

void RobotCommands::generateStairClimbList(int numSteps)
{
	ROS_INFO("Starting Stair Climb List Generation");
	stairClimbList.default_swing_time = 1.5; //swingTime;
	stairClimbList.default_transfer_time = 0.8; //transferTime;
	stairClimbList.execution_mode = 0 ; //3 = obstacle avoidance
	stairClimbList.unique_id = 2;

	double stepRun = 0.2387;
	double stepRise = 0.2031;
	tf::Vector3 stepVec;
	for (int i = 0; i < numSteps; i++)
	{
		//first step offset was -0.09
		if (i == 0)
			stepVec.setValue(stepRun*(i+1)+0.05, 0, stepRise*(i+1)-0.11); //+0.07, 0, +0.04 working config
		else
			stepVec.setValue((stepRun+0.005)*(i+1)+0.031, 0, stepRise*(i+1)-0.11); // 0, 0, +0.01

		stairClimbList.footstep_data_list.push_back(RobotCommands::createStairStep(true, stepVec));
		stairClimbList.footstep_data_list.push_back(RobotCommands::createStairStep(false, stepVec));
	}
	numStepsTaken = 0;
	ROS_INFO("Stair Climb List Generated");
}

ihmc_msgs::FootstepDataRosMessage RobotCommands::createStairStep(bool isRight, tf::Vector3 offset)
{
	ihmc_msgs::FootstepDataRosMessage footstep;
	std::string footFrameName;
	if (isRight)
	{
		footstep.robot_side = 1;
		footFrameName = "rightFoot";
	}
	else
	{
		footstep.robot_side = 0;
	    footFrameName = "leftFoot";
	}

	tf::StampedTransform footFrame;

	try{
		tfListener.lookupTransform("world", footFrameName.c_str(), ros::Time(0), footFrame);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	footstep.orientation = this->convertQuat(footFrame.getRotation());
	footstep.location = this->convertVec(footFrame.getOrigin());

	tf::Vector3 newLocation = footFrame*offset;

	footstep.location = this->convertVec(newLocation);

	ROS_INFO("Added Footstep to %f, %f, %f", newLocation.getX(), newLocation.getY(), newLocation.getZ());

	ihmc_msgs::Point2dRosMessage contactPoint1;
	ihmc_msgs::Point2dRosMessage contactPoint2;
	ihmc_msgs::Point2dRosMessage contactPoint3;
	ihmc_msgs::Point2dRosMessage contactPoint4;
	contactPoint1.x = 0.11;
	contactPoint1.y = 0.07;
	contactPoint2.x = 0.11;
	contactPoint2.y = -0.07;
	contactPoint3.x = -0.02;
	contactPoint3.y = -0.07;
	contactPoint4.x = -0.02;
	contactPoint4.y = 0.07;
	footstep.predicted_contact_points.push_back(contactPoint1);
	footstep.predicted_contact_points.push_back(contactPoint2);
	footstep.predicted_contact_points.push_back(contactPoint3);
	footstep.predicted_contact_points.push_back(contactPoint4);

	footstep.swing_height = 0.15; // was 0.25

    return footstep;
}

void RobotCommands::generateFootstepList()
{
	//uint numWayPoints = walkPath.size();
	//tf::StampedTransform rightFootFrame;
	//tf::StampedTransform leftFootFrame;
	//tf::StampedTransform pelvisFrame;
	//try{
	//	tfListener.lookupTransform("world", "rightFoot", ros::Time(0), rightFootFrame);
	//	tfListener.lookupTransform("world", "leftFoot", ros::Time(0), leftFootFrame);
	//	tfListener.lookupTransform("world", "pelvis", ros::Time(0), pelvisFrame);
	//}
	//catch (tf::TransformException &ex){
	//	ROS_ERROR("%s",ex.what());
	//	ros::Duration(1.0).sleep();
	//}

	//tf::Matrix3x3 pelvisRotation = tf::Matrix3x3(pelvisFrame.getRotation());
	//double curHeadingDeg = atan2(pelvisRotation.getColumn(0).getY(),
	//		pelvisRotation.getColumn(0).getX())*57.29578;

	if (footStepList.footstep_data_list.size() > 0)
	{
		footStepList.footstep_data_list.clear();
	}

	footStepList.default_swing_time = swingTime;
	footStepList.default_transfer_time = transferTime;
	footStepList.execution_mode = 0 ; //3 = obstacle avoidance
	footStepList.unique_id = 2;

	centerFeet();

	tf::Vector3 worldXYZ;
	//double xDelta;
	//double yDelta;
	//double prevX;
	//double prevY;
//	for (uint i = 0; i < numWayPoints; i ++)
//	{
//		worldXYZ = getGroundIntersect3D(walkPath[i].x, walkPath[i].y);
//		if (i == 0)
//		{
//			xDelta = worldXYZ.getX() - pelvisFrame.getOrigin().getX();
//			yDelta = worldXYZ.getY() - pelvisFrame.getOrigin().getY();
//		}
//		else
//		{
//			xDelta = worldXYZ.getX() - prevX;
//			yDelta = worldXYZ.getY() - prevY;
//		}
//
//		double newHeadingDeg = atan2(yDelta,xDelta)*57.29578;
//		double walkDistance = sqrt(xDelta*xDelta + yDelta*yDelta);
//
//		double headingDeltaDeg = (newHeadingDeg - curHeadingDeg);
//
//		addHeadingChangeSteps(headingDeltaDeg);
//		addStraightSteps(walkDistance);
//
//		prevX = worldXYZ.getX();
//		prevY = worldXYZ.getY();
//		curHeadingDeg = newHeadingDeg;
//	}

	publishStepList();
}

ihmc_msgs::FootstepDataListRosMessage RobotCommands::getFootStepList()
{
	return footStepList;
}
