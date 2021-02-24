#include "yoloTracking/YoloTracker.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "yoloTracking");
    YoloTracker *yolotracker = new YoloTracker();
    ros::NodeHandle n;
    ros::Subscriber yoloBoundingBox = n.subscribe("/darknet_ros/bounding_boxes", 1000, &YoloTracker::yoloCallback, yolotracker);
    ros::spin();
}
