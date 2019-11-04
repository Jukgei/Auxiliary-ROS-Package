#include "../include/auxiliary/OpticalFlow.hpp"

auxiliary::OpticalFlow::OpticalFlow(bool Display, bool Save){
   //this->Cap = VideoCapture a(0);
    VideoCapture buff(0); 
    
    if(buff.set(CAP_PROP_FRAME_WIDTH,320)){
        std::cout<< "Caps set success. width:320"<<std::endl;
    }
    if(buff.set(CAP_PROP_FRAME_HEIGHT,240)){
        std::cout<< "Caps set success. Height: 240"<<std::endl;
    }
    if(buff.set((CAP_PROP_FOURCC), CV_FOURCC('M','J','P','G'))){
        std::cout<< "Caps set FOURCC success" <<std::endl;
    }
    if(buff.set(CAP_PROP_FPS,60)){
        std::cout<< "Caps set success. Fps: 60"<<std::endl;
    }
    this->Cap = buff;
    this->Save = Save;
    this->Display = Display;
    FrameId = 0;
    DisplayName = "OpticalFlow";
    //if(!Cap.isOpened()){
    //    std::cout<<"Cannot open camera."<<std::endl;
    //    return ;
    //}
    //if(Cap.set(CV_CAP_PROP_FPS,60)){
    //    std::cout<< "Caps set success. Fps: 60"<<std::endl;
    //}
    fps = Cap.get(CAP_PROP_FPS); //get fps
    std::cout<<"FPS:"<<fps<<std::endl;
    if(fps <= 0 )
        fps = 25;
    std::cout<<"Get width"<<Cap.get(CAP_PROP_FRAME_WIDTH)<<std::endl;
    std::cout<<"Get Height"<<Cap.get(CAP_PROP_FRAME_HEIGHT)<<std::endl;
    if(Save){
        vw.open("opticalflowvideo/opticalflow.avi",
                CV_FOURCC('M','J','P','G'),
                fps,
                Size((int)Cap.get(CAP_PROP_FRAME_WIDTH),
                     (int)Cap.get(CAP_PROP_FRAME_HEIGHT))
                );
        raw.open("opticalflowvideo/opticalflowRGB.avi",
                CV_FOURCC('M','J','P','G'),
                fps,
                Size((int)Cap.get(CAP_PROP_FRAME_WIDTH),
                     (int)Cap.get(CAP_PROP_FRAME_HEIGHT))
                );
        if(!vw.isOpened()){
            std::cout<<"Video write error!"<<std::endl;
        }
        if(!raw.isOpened()){
            std::cout<<"Video(RGB) write error!"<<std::endl;
        }
    }

    //Parameter
    MaxCorners = 500;
    QualityLevel = 0.3;
    MinDistance = 7.0;
    BlockSize = 7;
    k = 0.04;
    UseHarris = true;

    FrameId = 0;
    isFindFeature = true;
    DetectInterval = 5;
    
    TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS,10,0.03);
    TrackLen = 10;

    Displacement.x = 0;
    Displacement.y = 0;

    //PatchSize = 6;

    isNiceThreshold = 1;
}

auxiliary::OpticalFlow::~OpticalFlow(){
    Cap.release();
}

bool auxiliary::OpticalFlow::GetImage(){
    Mat buf;
    //Mat FrameYCrCb;
    if(Cap.read(buf)){
        buf.copyTo(FrameRGB);
        raw.write(FrameRGB);
        Width = FrameRGB.cols;
        Height = FrameRGB.rows;
        cvtColor(FrameRGB,FrameGray,CV_BGR2GRAY);
        
        if(Display || Save)
            FrameRGB.copyTo(Visualization);
        return true;
    }
    else
        return false;

}


std::string auxiliary::OpticalFlow::ReturnDisplayName(){
    return this->DisplayName;
}

Point2f auxiliary::OpticalFlow::OpticalTracking(){
    Mat img0 = FrameGrayPrev;
    Mat img1 = FrameGray;
    std::vector<Point2f> p0;
    std::vector<Point2f> p1;
    std::vector<Point2f> p0r;
    std::vector<uchar>  status;
    std::vector<float> err;
    std::vector<Point2f> d;
    std::vector<bool> isNice;
    std::vector<std::vector<Point2f>> NewTrackPoints;
    Size winSize(15,15);
    //TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS,10,0.03);

    for(uint32_t i = 0; i < TrackPoints.size(); i++){
        std::vector<Point2f> temp;
        temp = TrackPoints[i];
        //printf("(%f,%f)",temp[temp.size()-1].x,temp[temp.size()-1].y);
        //std::cout<<"temp"<<temp[temp.size()-1].x<<','<<temp[temp.size()-1].y<<' '<<std::endl;
        p0.push_back(temp[temp.size()-1]);
    }
    //for(uint32_t i = 0; i < p0.size(); i++)
    //    printf("(%f,%f)\n",p0[i].x,p0[i].y);
    //for(uint32_t i = 0; i < p0.size();i++)
    //    std::cout<<p0[i]<<std::endl;
    calcOpticalFlowPyrLK(img0,img1,
                         p0,p1,status,
                         err, winSize,
                         2,termcrit);

    calcOpticalFlowPyrLK(img1,img0,
                         p1,p0r,status,
                         err,winSize,
                         2,termcrit);
    PointVectorErr(p0,p0r,d); 
    
    uint32_t num = 0;
    Point2f SumErr(0.0,0.0);
    
    JudgmentPoint(d,isNice);
    std::vector<Point2f> Prev;
    std::vector<Point2f> Now;
    std::vector<std::vector<Point2f>> FirstJudgment;
    //First judgment is a very simple idea.
    for(uint32_t i = 0; i < p1.size() && num < 100; i++){
        if( !isNice[i] )
            continue;

        std::vector<Point2f> temp = TrackPoints[i];
        Prev.push_back(temp[temp.size()-1]);
        Now.push_back(p1[i]);
        SumErr = (p1[i]  - temp[temp.size()-1] ) + SumErr; 
        num++;
        temp.push_back(p1[i]);
        if(temp.size() > TrackLen){
            std::vector<Point2f>::iterator k = temp.begin();
            temp.erase(k); //Delet the first element
        }
        FirstJudgment.push_back(temp);
        //if(Display || Save)
        //    circle(Visualization,p1[i],2, Scalar(0,255,0), -1);
    }
    
    
    //Second Judgment use RANSAC algorithm.
    Displacement.x = SumErr.x/num;
    Displacement.y = SumErr.y/num; 
    this->RANSACThreshold = sqrt(Displacement.x * Displacement.x + 
                                 Displacement.y * Displacement.y) * 0.5;
    this->RANSACThreshold = this->RANSACThreshold > 1 ? this->RANSACThreshold : 1;
    this->isNiceThreshold = this->RANSACThreshold;
    
    std::vector<bool> isInner;
    Mat mask;
    Mat resMatrix;
    if(Prev.size() > 0 && Now.size() > 0){
        resMatrix = findHomography(Prev, Now, RANSAC,
                                   RANSACThreshold, mask,
                                   300, 0.995);
        mask = mask.reshape(1,1).clone();
        for(uint8_t i = 0; i < mask.cols; i++){
            if(mask.at<uchar>(0,i) != 0){
                isInner.push_back(true);
                NewTrackPoints.push_back(FirstJudgment[i]);
            }
            else
                isInner.push_back(false);
        }
    }
    
    //Get return value
    Point2f res;
    res.x = resMatrix.at<double>(0,2);
    res.y = resMatrix.at<double>(1,2);
    //Debug
    //std::cout<<"Matrix:"<<resMatrix<<std::endl;
    std::cout<<"Point:"<<res<<std::endl;


    //Update the trackpoints
    if(Display || Save)
        TrackPoints = FirstJudgment;
    else
        TrackPoints = NewTrackPoints;
    
    //Visulization
    if(Display || Save){
        std::vector<Point2i> DrawPointSet;
        for(uint32_t i = 0 ; i < FirstJudgment.size(); i++){
            std::vector<Point2f> temp = FirstJudgment[i];
            for(uint32_t i = 0; i < temp.size(); i++){
                //std::cout<<'('<<IntPoint.x<<","<<IntPoint.y<<')'<<' '<<std::endl;
                DrawPointSet.push_back(Point2i(temp[i]));
            }
            if(isInner[i]){
                circle(Visualization,DrawPointSet[DrawPointSet.size()-1],2, Scalar(0,255,0), -1);
                polylines(Visualization,DrawPointSet,false,Scalar(0,255,0));
                DrawPointSet.clear();
            }
            else{
                circle(Visualization,DrawPointSet[DrawPointSet.size()-1],2, Scalar(0,0,255), -1);
                polylines(Visualization,DrawPointSet,false,Scalar(0,0,255));
                DrawPointSet.clear();
            }
        }
    }
    if(Save)
        vw.write(Visualization);
    return res;
}

void auxiliary::OpticalFlow::JudgmentPoint(const std::vector<Point2f> err, std::vector<bool>  &isNice ){
    for(uint32_t i = 0; i < err.size(); i++){
        if(err[i].x < isNiceThreshold && err[i].y < isNiceThreshold)
            isNice.push_back(true);
        else
            isNice.push_back(false);
    }
}

void auxiliary::OpticalFlow::PointVectorErr(const std::vector<Point2f> a,const std::vector<Point2f> b,std::vector<Point2f> &d){
    for(uint32_t i = 0; i < a.size(); i++){
        Point2f temp = a[i] - b[i];
        if(temp.x < 0)
            temp.x = -temp.x;
        if(temp.y < 0)
            temp.y = -temp.y;
        d.push_back(temp);
    }
}

void auxiliary::OpticalFlow::FindFeaturePoints(){
    Mat mask = Mat::zeros(FrameRGB.rows,FrameRGB.cols,CV_8UC1);
    mask = 255; //Bug

    for( uint32_t i = 0; i < TrackPoints.size(); i++   ) {
        std::vector<Point2f> point = TrackPoints[i];
        Point2i temp = point[point.size()-1];
        circle(mask,temp,5,0,-1);
    }
    std::vector<Point2i> p; 
    goodFeaturesToTrack(FrameGray,
                        p,
                        MaxCorners,
                        QualityLevel,
                        MinDistance,
                        mask,
                        BlockSize,
                        UseHarris,
                        k);
    //DetectShadow(p);

    if(!p.empty()){
        //std::vector<Point2f> temp;
        //IntPointToFloat(p,temp);
        for(uint32_t i = 0; i < p.size(); i++ ){
            std::vector<Point2f> buf;
            buf.push_back(Point2f(p[i]));
            TrackPoints.push_back(buf);
            buf.clear();
        }
    }

}

void auxiliary::OpticalFlow::Update(){
    FrameId = (FrameId + 1) % DetectInterval;
    if(FrameId == 0)
        isFindFeature = true;
    else
        isFindFeature = false;
    
    FrameGray.copyTo(FrameGrayPrev);
    if(Display)
      imshow(ReturnDisplayName(),Visualization);
    
} 


int auxiliary::OpticalFlow::ReturnTrackPointsSize(){
    return TrackPoints.size();
}

bool auxiliary::OpticalFlow::ReturnisFindFeature(){
    return isFindFeature;
}

bool auxiliary::OpticalFlow::ReturnDisplay(){
    return Display;
}

