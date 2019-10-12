#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <vector>
#include <unistd.h>
#include "auxiliary/state.h"
#include "control/controls.h"
#include "../include/auxiliary/auxiliary.hpp"
#include "functional"

using namespace std;


auxiliary::auxiliaryNode::auxiliaryNode(ros::NodeHandle &n){
    this->InitPublishers(n);

    this->InitSubcribers(n);
    
    this->InitDataPackageThread(); 

    this->InitOptFlowThread();
}

void auxiliary::auxiliaryNode::InitPublishers(ros::NodeHandle &n){
    AuxiliaryPublisher = n.advertise<auxiliary::state>("state",10);
    //SetTest(50);
    SetPublishFrequency(50);
    std::thread pub(std::bind(&auxiliaryNode::Publish, this));

    pub.detach();    
}

void auxiliary::auxiliaryNode::InitDataPackageThread(){
    std::thread dat(std::bind(&auxiliaryNode::DataPackageThread,this));
    dat.detach();
}

void auxiliary::auxiliaryNode::InitOptFlowThread(){

}

void auxiliary::auxiliaryNode::InitSubcribers(ros::NodeHandle &n){
    ArmControlSubscriber = n.subscribe<control::controls>
        ("controls",10,&auxiliaryNode::GetArmControlsCallBack,this);
}

void auxiliary::auxiliaryNode::GetArmControlsCallBack(const control::controls::ConstPtr& msg){
    //Get Arm Position controls and time controls
    std::vector<uint16_t> ArmCtrPos = msg->armCtr;
    std::vector<uint16_t> ArmCtrTime = msg->timeCtr;
    if(msg->GripSta ==  0x01)
        this->myPackage.SetGripperSta(grasp);
    else if(msg->GripSta == 0x00)
        this->myPackage.SetGripperSta(loose);
    else 
        this->myPackage.SetGripperSta(stop);
    //Prepare control
    for(uint8_t i = 1; i <=5; i++){
        this->myArm[i].CtrPos(ArmCtrPos[i]);
        this->myArm[i].CtrTime(ArmCtrTime[i]);
    }
    
}

void auxiliary::auxiliaryNode::DataPackageThread(){
    
    std::string port("/dev/ttyUSB0");
    unsigned long baud= 115200;
    serial::Serial myserial(port, baud, serial::Timeout::simpleTimeout(2)); //block 2ms
    //Set Feedback Bit
    //for(uint8_t i = 1; i <= 5; i++)
    //    this->myPackage.GetArmPos(i);
    //this->myPackage.GetHeight();
    
    while(true){
        this->myPackage.ReceiveMsg(myserial,this->myArm);
        std::cout<<"Receive success"<<std::endl;
        this->myPackage.GroupFrames(this->myArm);
        std::cout<<"GroupFrames success"<<std::endl;
        this->myPackage.SendControlPackage(myserial);
        std::cout<<"SendControl"<<std::endl;
        //Don't clear feedback Bit
        usleep(10000); //10ms
    }

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
