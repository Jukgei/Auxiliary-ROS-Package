#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <vector>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "auxiliary/state.h"
#include "auxiliary/controls.h"
#include "auxiliary/opticalflow.h"
#include "../include/auxiliary/auxiliary.hpp"
#include "functional"


#include <chrono>


using namespace std;

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

auxiliary::auxiliaryNode::auxiliaryNode(ros::NodeHandle &n){
    
    myArm[1].SetID(1);
    myArm[1].CtrPos(500);
    myArm[1].CtrTime(50);


    myArm[2].SetID(2);
    //myArm[2].CtrPos(792);
    myArm[2].CtrPos(500);
    myArm[2].CtrTime(50);

    myArm[3].SetID(3);
    //myArm[3].CtrPos(251);
    myArm[3].CtrPos(131);
    myArm[3].CtrTime(50);
    
    myArm[4].SetID(4);
    //myArm[4].CtrPos(664);
    myArm[4].CtrPos(141);
    myArm[4].CtrTime(50);
   
    myArm[5].SetID(5);
    myArm[5].CtrPos(468);
    myArm[5].CtrTime(50);
    
    this->InitPublishers(n);
    
    this->InitSubcribers(n);
    
    this->InitDataPackageThread(); 

    this->InitOptFlowThread();
}

void auxiliary::auxiliaryNode::InitPublishers(ros::NodeHandle &n){
    AuxiliaryPublisher = n.advertise<auxiliary::state>("state",10);
    //SetTest(50);
    OptiFlowPublisher = n.advertise<auxiliary::opticalflow>("opticalflow",10);
     
    SetPublishFrequency(50);
    std::thread pub(std::bind(&auxiliaryNode::Publish, this));

    pub.detach();    
}

void auxiliary::auxiliaryNode::InitDataPackageThread(){
    std::thread dat(std::bind(&auxiliaryNode::DataPackageThread,this));
    dat.detach();
}

void auxiliary::auxiliaryNode::InitOptFlowThread(){
    std::thread Opt(std::bind(&auxiliaryNode::OpticalFlowThread,this));
    Opt.detach();
}

void auxiliary::auxiliaryNode::InitSubcribers(ros::NodeHandle &n){
    ArmControlSubscriber = n.subscribe<auxiliary::controls>
        ("controls",10,&auxiliaryNode::GetArmControlsCallBack,this);
    GripperControlSubscriber = n.subscribe<auxiliary::gripper>
        ("gripper",10,&auxiliaryNode::GripperControlsCallBack,this);
}

void auxiliary::auxiliaryNode::GetArmControlsCallBack(const auxiliary::controls::ConstPtr& msg){
    //Get Arm Position controls and time controls
    std::vector<uint16_t> ArmCtrPos = msg->armCtr;
    std::vector<uint16_t> ArmCtrTime = msg->timeCtr;
    //if(msg->GripSta ==  0x01)
    //    this->myPackage.SetGripperSta(grasp);
    //else if(msg->GripSta == 0x00)
    //    this->myPackage.SetGripperSta(loose);
    //else 
    //    this->myPackage.SetGripperSta(stop);
    //Prepare control
    for(uint8_t i = 1; i <=5; i++){
        this->myArm[i].CtrPos(ArmCtrPos[i]);
        this->myArm[i].CtrTime(ArmCtrTime[i]);
    }
    //DEBUG PRINT
    //for(int i = 1; i <= 5; i ++)
    //        printf("Arm[%d] Ctr Pos: %d, Ctr Time:%d \n",i,myArm[i].GetCtrPos(),myArm[i].GetCtrTime());
}

void auxiliary::auxiliaryNode::GripperControlsCallBack(const auxiliary::gripper::ConstPtr& msg){
    //Get Arm Position controls and time controls
    if(msg->GripSta ==  0x01){
        this->myPackage.SetGripperSta(grasp);
        std::cout<<"Grasp"<<std::endl;
    }
    else if(msg->GripSta == 0x00){
        this->myPackage.SetGripperSta(loose);
        std::cout<<"loose"<<std::endl;
    }
    else{
        this->myPackage.SetGripperSta(stop);
        std::cout<<"stop"<<std::endl;
    }
    //Prepare control
    
    //DEBUG PRINT
    //for(int i = 1; i <= 5; i ++)
    //        printf("Arm[%d] Ctr Pos: %d, Ctr Time:%d \n",i,myArm[i].GetCtrPos(),myArm[i].GetCtrTime());
}

void auxiliary::auxiliaryNode::DataPackageThread(){
    
    std::string port("/dev/ttyUSB0");
    unsigned long baud= 115200;
    serial::Serial myserial(port, baud, serial::Timeout::simpleTimeout(2)); //block 2ms
    //Set Feedback Bit
    //for(uint8_t i = 1; i <= 5; i++)
    //    this->myPackage.GetArmPos(i);
    //this->myPackage.GetHeight();
    
    ros::Rate LoopRate(100);
    while(ros::ok()){
        this->myPackage.ReceiveMsg(myserial,this->myArm);
        //std::cout<<"Receive success"<<std::endl;
        this->myPackage.GroupFrames(this->myArm);
        //std::cout<<"GroupFrames success"<<std::endl;
        this->myPackage.SendControlPackage(myserial);
        //std::cout<<"SendControl"<<std::endl;
        //Don't clear feedback Bit
        //usleep(10000); //10ms
        LoopRate.sleep();
    }

}

void auxiliary::auxiliaryNode::OpticalFlowThread(){
    auxiliary::OpticalFlow myOpticalFlow(false, false); //Two Parameter: isDisplay and isSave 
    //std::cout<<"optical start"<<std::endl;
    Point2f DeltaPosition;
    bool ShowRunTime = false;
    high_resolution_clock::time_point StartTime;
    high_resolution_clock::time_point EndTime;
    milliseconds TimeInterval;  
    ros::Rate LoopRate(50);
    while(true){
        if(ShowRunTime)
            StartTime = high_resolution_clock::now();
        if(!myOpticalFlow.GetImage()){
            continue;
        }
    
        if(myOpticalFlow.ReturnTrackPointsSize() >0){
            //std::cout<<"Tracking"<<std::endl;
            DeltaPosition = myOpticalFlow.OpticalTracking();
        }
        
        if(myOpticalFlow.ReturnisFindFeature()){
            //std::cout<<"Find Feature Point"<<std::endl;
            myOpticalFlow.FindFeaturePoints();
        }

        myOpticalFlow.Update();


        //publish position data
        std::vector<float> OpticalflowData;
        OpticalflowData.push_back(DeltaPosition.x);
        OpticalflowData.push_back(DeltaPosition.y);
        //std::cout<<"delta x is "<< DeltaPosition.x<<"  delta y is "<<DeltaPosition.y<<std::endl;
        auxiliary::opticalflow opt; 
        opt.displacement = OpticalflowData;
       
        //if(abs(DeltaPosition.x) > 400 && abs(DeltaPosition.y) > 400){
        //    std::cout<<"error!"<<std::endl;
        //}
        
        
        this->OptiFlowPublisher.publish(opt);

        if(myOpticalFlow.ReturnDisplay()){
            if(waitKey(5) == 'q')
                break;
        }
        //else
        //    usleep(5000);
        
        if(ShowRunTime){
            EndTime = high_resolution_clock::now();
            TimeInterval = std::chrono::duration_cast<milliseconds>(EndTime - StartTime); 
            std::cout<<"Run Time:"<<TimeInterval.count()<<"ms"<<std::endl;
        }
        LoopRate.sleep();
    }
    
    //destroyWindow(myOpticalFlow.ReturnDisplayName());
    std::cout<<"OpticalFlow Thread End."<<std::endl;
}


void auxiliary::auxiliaryNode::Publish(){
    auxiliary::state sta;
    ros::Rate LoopRate(PublishFrequency);
    while(ros::ok())
    {
        sta.height = (float)(this->myPackage.ReturnHeight())/10.0; 
        std::vector<uint16_t> ArmPos;
        ArmPos.push_back(0);    //PlaceHolder
        for(uint8_t i = 1; i <=5 ; i++)
            ArmPos.push_back(this->myArm[i].GetPos());
        sta.arm = ArmPos;
        this->AuxiliaryPublisher.publish(sta);
        LoopRate.sleep();
    }

}

void auxiliary::auxiliaryNode::SetPublishFrequency(uint16_t frequency){
    PublishFrequency = frequency;
}


//void auxiliary::auxiliaryNode::SetTest(int test){
//    this->test_num = test;
//}
//
//void auxiliary::auxiliaryNode::test(){
//    std::cout<<test_num<<std::endl;
//}
//
