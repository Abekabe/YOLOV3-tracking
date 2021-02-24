#ifndef YOLOTRACKER_H
#define YOLYTRACKER_H

#include "KalmanTracker.h"
#include "Hungarian.h"
#include <iostream>
#include <sstream>
#include <vector>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>


// For opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct ObjectTracker {
    KalmanTracker tracker;
    string cls;
    int serial;
};

class YoloTracker {
    private:
        vector<ObjectTracker> objectVector;
        //vector<vector<double> > predictMatrix;
        double currentTime, previousTime;
        int seq, number;
        bool init;
        image_transport::Subscriber image_sub_;
        image_transport::ImageTransport it_;
        ros::NodeHandle nh_;
        vector<cv::Scalar> randomColor;

    public:
        YoloTracker();
        ~YoloTracker();
        vector<int> getAssociation(vector<vector<double> >&, vector<vector<double> >&);
        vector<vector<double> > predictState(ros::Time&);
        void updateState(ros::Time&, vector<vector<double> >&, vector<string>&);
        vector<double> getLossVector(vector<double>&, vector<vector<double> >&);
        void yoloCallback(const darknet_ros_msgs::BoundingBoxes& msg);
        void incNumber();
        void pubImage(const sensor_msgs::ImageConstPtr&);
};


#endif
