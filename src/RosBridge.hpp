#ifndef ROSBRIDGE_HPP_
#define ROSBRIDGE_HPP_

#include <ros/ros.h>
#include "GetImageService.h"

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

    bool GetCameraImage(bridge::GetImageService::Request&, bridge::GetImageService::Response&);

    void StoreCameraImage(CAMERA_ID, const sensor_msgs::Image::ConstPtr&);
    void StoreLeftHeadImage(const sensor_msgs::Image::ConstPtr&);
    static void* ThreadTarget(void*);
    void AddNano(struct timespec&, int64_t);

    ros::Subscriber leftHeadImageSubscriber;
    ros::ServiceServer getCameraImageService;

    ros::NodeHandle *nodeHandle;

    sensor_msgs::Image *imageArray;
    static unsigned int referenceCount;

    pthread_mutex_t *mutex;
    bool runThread;
    pthread_t thread;
    int64_t nanoPeriod;
};

#endif
