#include "../include/auxiliary/OpticalFlow.hpp"

auxiliary::OpticalFlow::OpticalFlow(bool Display){
   //this->Cap = VideoCapture a(0);
    VideoCapture buff(0);
    this->Cap = buff;
    this->Display = Display;
    FrameId = 0;
    DisplayName = "OpticalFlow";
    if(!Cap.isOpened()){
        std::cout<<"Cannot open camera."<<std::endl;
        return ;
    }

    //Parameter
    MaxCorners = 500;
    QualityLevel = 0.3;
    MinDistance = 7.0;
    BlockSize = 7;
    k = 0.04;
    UseHarris = true;

    FrameId = 0;
}


void auxiliary::OpticalFlow::GetImage(){
    Cap >> FrameRGB;
    cvtColor(FrameRGB,FrameGray,CV_BGR2GRAY);
}


std::string auxiliary::OpticalFlow::ReturnDisplayName(){
    return this->DisplayName;
}

void auxiliary::OpticalFlow::FindFeaturePoints(){
    Mat mask = Mat::zeros(FrameRGB.rows,FrameRGB.cols,CV_8UC1);
    mask = 255; //Bug

    for( int i = 0; i < TrackPoints.size(); i++   ) {
        Point2f point = TrackPoints[i];
        circle(mask,point,5,0,-1);
    }
    std::vector<Point2f> p; 
    goodFeaturesToTrack(FrameGray,
                        p,
                        MaxCorners,
                        QualityLevel,
                        MinDistance,
                        mask,
                        BlockSize,
                        UseHarris,
                        k);
    if(!p.empty()){
        for(int i = 0; i < p.size(); i++ ){
            TrackPoints.push_back(p[i]);
        }
    }
}


