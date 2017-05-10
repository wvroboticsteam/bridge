#include "RosBridge.hpp"

unsigned int RosBridge::referenceCount = 0;

RosBridge::RosBridge()
{
    if(referenceCount == 0)
    {
	if(!ros::isInitialized())
	{
	    char **argv = new char*;
	    *argv = new char[7];
	    snprintf(argv[0], 7, "%s", "bridge");
	    argv[6] = 0;
	    int argc = 1;

	    ros::init(argc, argv, "bridge");

	    delete[] argv[0];
	    delete argv;
	}

	imageArray = new sensor_msgs::Image[CAMERA_ID_SIZE];
	
	nodeHandle = new ros::NodeHandle;

	leftHeadImageSubscriber = nodeHandle->subscribe("/multisense/camera/left/image_color", 1, &RosBridge::StoreLeftHeadImage, this);
	getCameraImageService = nodeHandle->advertiseService("/bridge/getCameraImage", &RosBridge::GetCameraImage, this);

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
	ros::shutdown();

	pthread_mutex_destroy(mutex);

	delete nodeHandle;
	delete[] imageArray;
	delete mutex;
    }
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
