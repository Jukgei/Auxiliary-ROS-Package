#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;

namespace auxiliary{

class OpticalFlow{
public:
    OpticalFlow(bool Display);
    void GetImage();
    std::string ReturnDisplayName();
    void FindFeaturePoints();


private:
    bool Display;
    Mat FrameRGB;
    Mat FrameGray;
    Mat FrameGrayPrev;
    

    VideoCapture Cap; 
    std::string DisplayName;
    std::vector<Point2f> TrackPoints;


    uint8_t FrameId;

    //Parameters
    int MaxCorners;
    double QualityLevel;
    double MinDistance;
    int BlockSize;
    double k;
    bool UseHarris;
};

}




#endif


