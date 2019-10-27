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
    fps = Cap.get(CAP_PROP_FPS); //get fps
    std::cout<<"FPS:"<<fps<<std::endl;
    if(fps <= 0 )
        fps = 25;
    vw.open("~/Documents/opticalflow.mp4",
            VideoWriter::fourcc('M','P','4','V'),
            fps,
            Size(Cap.get(CAP_PROP_FRAME_WIDTH)/2,
                 Cap.get(CAP_PROP_FRAME_HEIGHT)/2)
            );
    if(!vw.isOpened()){
        std::cout<<"Video write error!"<<std::endl;
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
}

auxiliary::OpticalFlow::~OpticalFlow(){
    Cap.release();
}

bool auxiliary::OpticalFlow::GetImage(){
    Mat buf;
    if(Cap.read(buf)){
        resize(buf, FrameRGB, Size(buf.cols/2,buf.rows/2),0,0,INTER_LINEAR);
        cvtColor(FrameRGB,FrameGray,CV_BGR2GRAY);
        if(Display)
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
    //for(uint32_t i = 0; i < status.size(); i++)
    //    printf("%d\n",status[i]);
    //for(uint32_t i = 0; i < p1.size(); i++)
    //    printf("(%f,%f)\n",p1[i].x,p1[i].y);
    //for(uint32_t i = 0; i < d.size(); i++)
    //    printf("(%f,%f)\n",d[i].x,d[i].y);
    //for(uint32_t i = 0; i < p0.size(); i++){
    //    //std::cout<<'('<<p0[i].x<<','<<p0[i].y<<')'<<
    //    //    ' '<<'('<<p0r[i].x<<','<<p0r[i].y<<')'<<std::endl;
    //    printf("p0:(%f,%f),p0r:(%f,%f)\n",p0[i].x,p0[i].y,p0r[i].x,p0r[i].y);
    //}

    uint32_t num = 0;
    Point2f SumErr(0.0,0.0);
    JudgmentPoint(d,isNice);
    for(uint32_t i = 0; i < p1.size(); i++){
        if( !isNice[i] )
            continue;

        std::vector<Point2f> temp = TrackPoints[i];
        SumErr = (p1[i]  - temp[temp.size()-1] ) + SumErr; 
        num++;
        temp.push_back(p1[i]);
        if(temp.size() > TrackLen){
            std::vector<Point2f>::iterator k = temp.begin();
            temp.erase(k); //Delet the first element
        }
        NewTrackPoints.push_back(temp);
        if(Display)
            circle(Visualization,p1[i],2, Scalar(0,255,0), -1);
    }
    Displacement.x = SumErr.x/num;
    Displacement.y = SumErr.y/num; 
    //if(std::isnan(Displacement.x) || std::isnan(Displacement.y))
    //    printf("num = %d\n SumErr(%f,%f)",num,SumErr.x,SumErr.y);
    //std::cout<<"("<<Displacement.x<<','<<Displacement.y<<')'<<std::endl;
    //Now Publish 
    TrackPoints = NewTrackPoints;
    //std::cout<<"TrackPoints Num :"<<TrackPoints.size()<<std::endl;
    if(Display){
        std::vector<Point2i> DrawPointSet;
        for(uint32_t i = 0 ; i < TrackPoints.size(); i++){
            std::vector<Point2f> temp = TrackPoints[i];
            for(uint32_t i = 0; i < temp.size(); i++){
                //std::cout<<'('<<IntPoint.x<<","<<IntPoint.y<<')'<<' '<<std::endl;
                DrawPointSet.push_back(Point2i(temp[i]));
            }
            //for(uint32_t i = 0; i < DrawPointSet.size();i++)
            //    printf("(%d,%d)\n",DrawPointSet[i].x,DrawPointSet[i].y);
            //std::cout<<'\n';
            //std::cout<<"DrawPointSetsize"<<DrawPointSet.size()<<std::endl;
            //printf("\n");
            polylines(Visualization,DrawPointSet,false,Scalar(0,255,0));
            DrawPointSet.clear();
        }
    }
    vw.write(Visualization);
    return Displacement;
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

//void auxiliary::OpticalFlow::IntPointToFloat( const std::vector<Point2i> i, std::vector<Point2f> &f ){
//    for(uint32_t j = 0; j < i.size(); j++ ){
//        f.push_back(Point2f(i[j]));
//    }
//}
//
//
