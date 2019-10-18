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
    isFindFeature = true;
    DetectInterval = 5;
    
    TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS,10,0.03);
    TrackLen = 10;
}


void auxiliary::OpticalFlow::GetImage(){
    Cap >> FrameRGB;
    cvtColor(FrameRGB,FrameGray,CV_BGR2GRAY);
    if(Display)
        FrameRGB.copyTo(Visualization);
}


std::string auxiliary::OpticalFlow::ReturnDisplayName(){
    return this->DisplayName;
}

void auxiliary::OpticalFlow::OpticalTracking(){
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
        //Point2f help(0.0001,0.0001);
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
    
    //for(uint32_t i = 0; i < status.size(); i++)
    //    printf("%d\n",status[i]);
    //for(uint32_t i = 0; i < d.size(); i++)
    //    printf("(%f,%f)\n",d[i].x,d[i].y);
    //for(uint32_t i = 0; i < p0.size(); i++){
    //    //std::cout<<'('<<p0[i].x<<','<<p0[i].y<<')'<<
    //    //    ' '<<'('<<p0r[i].x<<','<<p0r[i].y<<')'<<std::endl;
    //    printf("p0:(%f,%f),p0r:(%f,%f)\n",p0[i].x,p0[i].y,p0r[i].x,p0r[i].y);
    //}

    JudgmentPoint(d,isNice);
    for(uint32_t i = 0; i < p1.size(); i++){
        if( !isNice[i] )
            continue;
        std::vector<Point2f> temp = TrackPoints[i];
        temp.push_back(p1[i]);
        if(temp.size() > TrackLen){
            std::vector<Point2f>::iterator k = temp.begin();
            temp.erase(k); //Delet the first element
        }
        NewTrackPoints.push_back(temp);
        if(Display)
            circle(Visualization,p1[i],2, (0,255,0), -1);
    }
    TrackPoints = NewTrackPoints;
    if(Display){
        std::vector<Point2i> DrawPointSet;
        for(uint32_t i = 0 ; i < TrackPoints.size(); i++){
            std::vector<Point2f> temp = TrackPoints[i];
            for(uint32_t i = 0; i < temp.size(); i++){
                Point2i IntPoint = temp[i];
                //std::cout<<'('<<IntPoint.x<<","<<IntPoint.y<<')'<<' '<<std::endl;
                DrawPointSet.push_back(IntPoint);
            }
            //std::cout<<'\n';
            //std::cout<<"DrawPointSetsize"<<DrawPointSet.size()<<std::endl;
            polylines(Visualization,DrawPointSet,false,(0,255,0));
            DrawPointSet.clear();
        }
    }
    
}

void auxiliary::OpticalFlow::JudgmentPoint(const std::vector<Point2f> err, std::vector<bool>  &isNice ){
    for(uint32_t i = 0; i < err.size(); i++){
        if(err[i].x < 1 && err[i].y <1)
            isNice.push_back(true);
        else
            isNice.push_back(false);
    }
}
void auxiliary::OpticalFlow::PointVectorErr(const std::vector<Point2f> a,const std::vector<Point2f> b,std::vector<Point2f> &d){
    for(uint16_t i = 0; i < a.size(); i++){
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

    if(!p.empty()){
        std::vector<Point2f> temp;
        IntPointToFloat(p,temp);
        for(uint32_t i = 0; i < temp.size(); i++ ){
            std::vector<Point2f> buf;
            buf.push_back(temp[i]);
            //printf("(%f,%f)",temp[i].x,temp[i].y);
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
    FrameGrayPrev = FrameGray;
    if(Display)
        imshow(ReturnDisplayName(),Visualization);

} 

int auxiliary::OpticalFlow::ReturnTrackPointsSize(){
    return TrackPoints.size();
}

bool auxiliary::OpticalFlow::ReturnisFindFeature(){
    return isFindFeature;
}

void auxiliary::OpticalFlow::IntPointToFloat( const std::vector<Point2i> i, std::vector<Point2f> &f ){
    for(uint32_t j = 0; j < i.size(); j++ ){
        f.push_back(Point2f(i[j]));
    }
}