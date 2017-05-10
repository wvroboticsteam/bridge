#ifndef ROSBRIDGE_HPP_
#define ROSBRIDGE_HPP_

#include <ros/ros.h>
#include "GetImageService.h"

class RosBridge
{
public:
    RosBridge();
    ~RosBridge();

protected:
    enum CAMERA_ID
    {
	LEFT_HEAD_CAMERA,
	CAMERA_ID_SIZE
    };

    bool GetCameraImage(bridge::GetImageService::Request&, bridge::GetImageService::Response&);

    void StoreCameraImage(CAMERA_ID, const sensor_msgs::Image::ConstPtr&);
    void StoreLeftHeadImage(const sensor_msgs::Image::ConstPtr&);

    ros::Subscriber leftHeadImageSubscriber;
    ros::ServiceServer getCameraImageService;

    ros::NodeHandle *nodeHandle;

    sensor_msgs::Image *imageArray;
    static unsigned int referenceCount;

    pthread_mutex_t *mutex;
};

#endif
