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
    OpticalFlow(bool Display, bool Save);
    bool GetImage();
    std::string ReturnDisplayName();
    void FindFeaturePoints();
    Point2f OpticalTracking();
    void Update();
    void PointVectorErr(const std::vector<Point2f> a,const std::vector<Point2f> b,std::vector<Point2f> &d); 
    void JudgmentPoint(const std::vector<Point2f> err, std::vector<bool>  &isNice);
    int ReturnTrackPointsSize();
    bool ReturnisFindFeature();
    bool ReturnDisplay();
    //void DetectShadow(const std::vector<Point2i> featurepoint);
    //void IntPointToFloat(const std::vector<Point2i> i, std::vector<Point2f> &f);
    ~OpticalFlow();

private:
    bool Display;
    bool Save;
    Mat FrameRGB;
    Mat FrameGray;
    Mat FrameGrayPrev;
    Mat Visualization; 
    //Mat Y;
    //Mat FrameBin;

    VideoCapture Cap; 
    std::string DisplayName;
    std::vector<std::vector<Point2f>> TrackPoints;
    
    //std::vector<Point2i> NicePoint;

    //Some variable about video write
    int fps;
    VideoWriter vw;
    VideoWriter raw;

    uint8_t FrameId;
    bool isFindFeature;
    uint8_t DetectInterval;

    //Parameters
    int MaxCorners;
    double QualityLevel;
    double MinDistance;
    int BlockSize;
    double k;
    bool UseHarris;
   
    
    TermCriteria termcrit;

    uint8_t TrackLen;
    
    //int PatchSize;
    int Width;
    int Height;
    //int LumenMean;
    Point2f Displacement;

    //check good point parameter
    double isNiceThreshold;   //first easy Judgment parameter
    double RANSACThreshold;   //RANSAC algorithm threshold
};

}




#endif


