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

	leftHeadImageSubscriber = nodeHandle->subscribe("/multisense/camera/left/image_color", 1, &RosBridge::StoreLeftHeadImage, this);
	getCameraImageService = nodeHandle->advertiseService(bridge::GET_CAMERA_IMAGE_SRV_NAME, &RosBridge::GetCameraImage, this);

	depthMapSubscriber = nodeHandle->subscribe("/multisense/camera/points2", 1, &RosBridge::StoreDepthMap, this);
	getDepthMapService = nodeHandle->advertiseService(bridge::GET_DEPTH_MAP_SRC_NAME, &RosBridge::GetDepthMap, this);

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
