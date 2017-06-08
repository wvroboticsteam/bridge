#include "RosBridge.hpp"
#include "BridgeTypes.hpp"

unsigned int RosBridge::referenceCount = 0;

RosBridge::RosBridge()
:runThread(false)
,nanoPeriod(10000000)
{
    if(referenceCount == 0)
    {
	if(!ros::isInitialized())
	{
	    char **argv = new char*;
	    *argv = new char[7];
	    snprintf(argv[0], 7, "%s", "bridge");
	    argv[0][6] = 0;
	    int argc = 1;

	    ros::init(argc, argv, "bridge");

	    delete[] argv[0];
	    delete argv;
	}

	imageArray = new sensor_msgs::Image[CAMERA_ID_SIZE];
	
	nodeHandle = new ros::NodeHandle;

	rc = new RobotCommands;

	InitializePublishers();
	InitializeSubscribers();
	InitializeServices();

	feetInitialized = false;


	mutex = new pthread_mutex_t;
	pthread_mutex_init(mutex, NULL);
    }

    referenceCount++;
}

RosBridge::~RosBridge()
{
    referenceCount--;

    if(referenceCount == 0)
    {
	if(runThread)
	{
	    runThread = false;
	    pthread_join(thread, NULL);
	}

	ros::shutdown();

	pthread_mutex_destroy(mutex);

	delete nodeHandle;
	delete[] imageArray;
	delete mutex;
    }
}

void RosBridge::InitializePublishers()
{
	footstepListPublisher = nodeHandle->advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list",1,true);
	handTrajPublisher = nodeHandle->advertise<ihmc_msgs::HandTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/hand_trajectory",1,true);
	armTrajPublisher = nodeHandle->advertise<ihmc_msgs::ArmTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/arm_trajectory",1,true);
	torsoPublisher = nodeHandle->advertise<ihmc_msgs::ChestTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/chest_trajectory",1,true);
	pelvisHeightPublisher = nodeHandle->advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>("ihmc_ros/valkyrie/control/pelvis_height_trajectory",1,true);
	headPublisher = nodeHandle->advertise<ihmc_msgs::HeadTrajectoryRosMessage>("ihmc_ros/valkyrie/control/head_trajectory",1,true);
}

void RosBridge::InitializeSubscribers()
{
	depthMapSubscriber = nodeHandle->subscribe("/multisense/camera/points2", 1, &RosBridge::StoreDepthMap, this);
	leftHeadImageSubscriber = nodeHandle->subscribe("/multisense/camera/left/image_color", 1, &RosBridge::StoreLeftHeadImage, this);
	leftImageInfoSubscriber = nodeHandle->subscribe("/multisense/camera/left/camera_info",1,&RosBridge::GetImageInfo,this);
	footstepStatusSubscriber = nodeHandle->subscribe("/ihmc_ros/valkyrie/output/footstep_status",1,&RosBridge::GetFootstepStatus,this);
	walkingStatusSubscriber = nodeHandle->subscribe("/ihmc_ros/valkyrie/output/walking_status",1,&RosBridge::GetWalkingStatus,this);
	highLevelStatusSubscriber = nodeHandle->subscribe("/ihmc_ros/valkyrie/output/high_level_state_change",1,&RosBridge::GetStateChange,this);
}

void RosBridge::InitializeServices()
{
	getCameraImageService = nodeHandle->advertiseService(bridge::GET_CAMERA_IMAGE_SRV_NAME, &RosBridge::GetCameraImage, this);
	getDepthMapService = nodeHandle->advertiseService(bridge::GET_DEPTH_MAP_SRV_NAME, &RosBridge::GetDepthMap, this);
	basicCommandService = nodeHandle->advertiseService(bridge::BASIC_COMMAND_SRV_NAME, &RosBridge::BasicCommands, this);
}

bool RosBridge::BasicCommands(bridge::BasicCommands::Request &req, bridge::BasicCommands::Response &res)
{

	bool result = true;
	if (req.commandString.data == "centerFeet")
	{
		ROS_INFO("Centering Feet");
		rc->centerFeet();
		this->PublishFootStepList();
	}
	else if (req.commandString.data == "closeFeet")
	{
		ROS_INFO("Closing Feet");
		rc->closeFeet();
		this->PublishFootStepList();
	}
	else if (req.commandString.data == "torsoLeanForward")
	{
		ROS_INFO("Leaning Torso Forward");
		this->PublishChestMotion(rc->generateTorsoMove(true));
	}
	else if (req.commandString.data == "torsoLeanBack")
	{
		ROS_INFO("Leaning Torso Back");
		this->PublishChestMotion(rc->generateTorsoMove(false));
	}
	else if (req.commandString.data == "centerFeet")
	{

	}
	else if (req.commandString.data == "centerFeet")
	{

	}
	else
	{
		ROS_INFO("Unknown Command");
		result = false;
	}



    pthread_mutex_lock(mutex);
    res.commandExecuted = true;
    pthread_mutex_unlock(mutex);

    return result;
}


bool RosBridge::GetDepthMap(bridge::GetDepthMap::Request&, bridge::GetDepthMap::Response &res)
{
    pthread_mutex_lock(mutex);
    res.pointCloud = depthMap;
    pthread_mutex_unlock(mutex);

    return true;
}

void RosBridge::StoreDepthMap(const sensor_msgs::PointCloud2::ConstPtr &ptr)
{
    pthread_mutex_lock(mutex);
    depthMap = *ptr;
    pthread_mutex_unlock(mutex);
}

void RosBridge::StoreCameraImage(CAMERA_ID id, const sensor_msgs::Image::ConstPtr &ptr)
{
    if(id == CAMERA_ID_SIZE)
	return;

    pthread_mutex_lock(mutex);
    imageArray[id] = *ptr;
    pthread_mutex_unlock(mutex);
}

void RosBridge::StoreLeftHeadImage(const sensor_msgs::Image::ConstPtr &ptr)
{
    StoreCameraImage(LEFT_HEAD_CAMERA, ptr);
}

bool RosBridge::GetCameraImage(bridge::GetImageService::Request &req, bridge::GetImageService::Response &res)
{
    if(req.cameraIndex >= CAMERA_ID_SIZE)
	return false;

    pthread_mutex_lock(mutex);
    res.cameraImage = imageArray[req.cameraIndex];
    pthread_mutex_unlock(mutex);

    return true;
}

void RosBridge::GetFootstepStatus(const ihmc_msgs::FootstepStatusRosMessage & footstepStatus)
{
	if (footstepStatus.status == 1)
	{
		numStepsTaken++;
		ROS_INFO("Completed Step %d", numStepsTaken);
	}
}

void RosBridge::GetImageInfo(const sensor_msgs::CameraInfo& newCameraInfo)
{
	fx = newCameraInfo.P[0];
	fy = newCameraInfo.P[5];
	Tx = newCameraInfo.P[3];
	Ty = newCameraInfo.P[7];
	cx = newCameraInfo.P[2];
	cy = newCameraInfo.P[6];
}

void RosBridge::GetStateChange(const ihmc_msgs::HighLevelStateChangeStatusRosMessage & stateChange)
{
	if (stateChange.initial_state == stateChange.WALKING && stateChange.end_state == stateChange.DO_NOTHING_BEHAVIOR)
	{
		ROS_INFO("Your ass fell over!");
	}
}

void RosBridge::GetWalkingStatus(const ihmc_msgs::WalkingStatusRosMessage & walkingStatus)
{
	if (walkingStatus.status == 1)
	{
		stillWalking = false;
		ROS_INFO("Done Walking!");
	}
}

void RosBridge::PublishFootStepList()
{
	footstepListPublisher.publish(rc->getFootStepList());
}

void RosBridge::PublishChestMotion(ihmc_msgs::ChestTrajectoryRosMessage chestMotion)
{
	torsoPublisher.publish(chestMotion);
}

bool RosBridge::RunLoopThread(int64_t nPeriod)
{
    int ret = 0;

    if(runThread)
	return false;

    nanoPeriod = nPeriod;
    runThread = true;
    if((ret = pthread_create(&thread, NULL, &RosBridge::ThreadTarget, this)) != 0)
	runThread = false;

    return (ret == 0);
}

void* RosBridge::ThreadTarget(void *data)
{
    RosBridge *parent = (RosBridge*)data;
    struct timespec ts;
    int sanityCheck;

    ROS_INFO("Ros thread entered\n");

    clock_gettime(CLOCK_MONOTONIC, &ts);
    while(parent->runThread && ros::ok())
    {
	sanityCheck = 0;

	ros::spinOnce();

	parent->AddNano(ts, parent->nanoPeriod);
	while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) != 0)
	    if(sanityCheck++ > 10)
		break;

    }

    ROS_INFO("Ros thread exited (runThread: %s | ros::ok: %s)\n", parent->runThread ? "YES" : "NO", ros::ok() ? "YES" : "NO");

    return NULL;
}

void RosBridge::AddNano(struct timespec &ts, int64_t nPeriod)
{
    const int64_t NANO_SEC = 1000000000;

    ts.tv_nsec += nPeriod;
    if(ts.tv_nsec > (NANO_SEC - 1))
    {
	ts.tv_sec += ts.tv_nsec / NANO_SEC;
	ts.tv_nsec = ts.tv_nsec % NANO_SEC;
    }
}
