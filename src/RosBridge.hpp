#ifndef ROSBRIDGE_HPP_
#define ROSBRIDGE_HPP_

#include <ros/ros.h>
#include "BasicCommands.h"
#include "GetImageService.h"
#include "GetDepthMap.h"
#include "MouseCommand.h"
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
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include "RobotCommands.hpp"

class RosBridge
{
public:
    RosBridge();
    ~RosBridge();

    bool RunLoopThread(int64_t);

protected:
    enum CAMERA_ID
    {
	LEFT_HEAD_CAMERA,
	CAMERA_ID_SIZE
    };

	void InitializeSubscribers();
	void InitializePublishers();
	void InitializeServices();

    bool GetCameraImage(bridge::GetImageService::Request&, bridge::GetImageService::Response&);

    void StoreCameraImage(CAMERA_ID, const sensor_msgs::Image::ConstPtr&);
    void StoreLeftHeadImage(const sensor_msgs::Image::ConstPtr&);
    static void* ThreadTarget(void*);
    void AddNano(struct timespec&, int64_t);
    void StoreDepthMap(const sensor_msgs::PointCloud2::ConstPtr&);
    bool GetDepthMap(bridge::GetDepthMap::Request&, bridge::GetDepthMap::Response&);
    bool BasicCommands(bridge::BasicCommands::Request&, bridge::BasicCommands::Response&);
    bool MouseCommand(bridge::MouseCommand::Request&, bridge::MouseCommand::Response&);

	void GetFootstepStatus(const ihmc_msgs::FootstepStatusRosMessage&);
	void GetWalkingStatus(const ihmc_msgs::WalkingStatusRosMessage&);
	void GetStateChange(const ihmc_msgs::HighLevelStateChangeStatusRosMessage&);
    void GetImageInfo(const sensor_msgs::CameraInfo&);

	ros::Publisher footstepListPublisher;
	ros::Publisher armTrajPublisher;
	ros::Publisher pelvisHeightPublisher;
	ros::Publisher torsoPublisher;
	ros::Publisher handTrajPublisher;
	ros::Publisher headPublisher;

	ros::Subscriber leftImageInfoSubscriber;
	ros::Subscriber footstepStatusSubscriber;
	ros::Subscriber walkingStatusSubscriber;
	ros::Subscriber highLevelStatusSubscriber;

    ros::Subscriber leftHeadImageSubscriber;
    ros::Subscriber depthMapSubscriber;

    ros::ServiceServer basicCommandService;
    ros::ServiceServer getCameraImageService;
    ros::ServiceServer getDepthMapService;
    ros::ServiceServer mouseCommandService;

    tf::TransformListener *tfListener;

    ros::NodeHandle *nodeHandle;

    RobotCommands *rc;

    void PublishFootStepList();
    void PublishStairFootStepList();
    void PublishChestMotion(ihmc_msgs::ChestTrajectoryRosMessage);
    void PublishArmMotion(ihmc_msgs::ArmTrajectoryRosMessage);

    sensor_msgs::Image *imageArray;
    static unsigned int referenceCount;
    sensor_msgs::PointCloud2 depthMap;
    tf::StampedTransform cameraOpticTransform;
    std::vector<int> mousePointsX;
    std::vector<int> mousePointsY;

    pthread_mutex_t *mutex;
    bool runThread;
    pthread_t thread;
    int64_t nanoPeriod;

	int numStepsTaken;

	double fx;
	double fy;
	double Tx;
	double Ty;
	double cx;
	double cy;
	bool feetInitialized;
	bool stillWalking;
};

#endif
