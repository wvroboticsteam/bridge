#ifndef ROSBRIDGE_HPP_
#define ROSBRIDGE_HPP_

#include <ros/ros.h>
#include "GetImageService.h"
#include "GetDepthMap.h"

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
    void StoreDepthMap(const sensor_msgs::PointCloud2::ConstPtr&);
    bool GetDepthMap(bridge::GetDepthMap::Request&, bridge::GetDepthMap::Response&);

    ros::Subscriber leftHeadImageSubscriber;
    ros::ServiceServer getCameraImageService;
    ros::Subscriber depthMapSubscriber;
    ros::ServiceServer getDepthMapService;

    ros::NodeHandle *nodeHandle;

    sensor_msgs::Image *imageArray;
    static unsigned int referenceCount;
    sensor_msgs::PointCloud2 depthMap;

    pthread_mutex_t *mutex;
    bool runThread;
    pthread_t thread;
    int64_t nanoPeriod;
};

#endif
