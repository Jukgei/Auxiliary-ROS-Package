#ifndef _AUXILIARY_H
#define _AUXILIARY_H


#include <serial.h>
#include <v8stdint.h>
#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <vector>
#include <functional>

#include "OpticalFlow.hpp"
#include "auxiliary/controls.h"
#include "auxiliary/gripper.h"

#define FIRST_HEADER            0
#define SECOND_HEADER           1
#define FIRST_ARM_POS_CTR_L     2
#define FIRST_ARM_POS_CTR_H     3
#define FIRST_ARM_TIM_CTR_L     4
#define FIRST_ARM_TIM_CTR_H     5
#define SECOND_ARM_POS_CTR_L    6
#define SECOND_ARM_POS_CTR_H    7
#define SECOND_ARM_TIM_CTR_L    8
#define SECOND_ARM_TIM_CTR_H    9
#define THIRD_ARM_POS_CTR_L     10
#define THIRD_ARM_POS_CTR_H     11
#define THIRD_ARM_TIM_CTR_L     12
#define THIRD_ARM_TIM_CTR_H     13
#define FOURTH_ARM_POS_CTR_L    14
#define FOURTH_ARM_POS_CTR_H    15 
#define FOURTH_ARM_TIM_CTR_L    16
#define FOURTH_ARM_TIM_CTR_H    17 
#define FIFTH_ARM_POS_CTR_L     18
#define FIFTH_ARM_POS_CTR_H     19
#define FIFTH_ARM_TIM_CTR_L     20
#define FIFTH_ARM_TIM_CTR_H     21
#define GRIPPER_CTR             22
//#define GIMBAL_YAW_CTR_L        23
//#define GIMBAL_YAW_CTR_H        24
//#define GIMBAL_PITCH_CTR_L      25
//#define GIMBAL_PITCH_CTR_H      26
//#define FB_CTR                  27
#define FIRST_TAIL              23
#define SECOND_TAIL             24


namespace auxiliary {

enum GripperStatus{
    loose = 0,
    grasp = 1,
    stop 
};


class arm{
public:
    //arm(uint8_t num);
    void SetID(uint8_t ID);
    void SetPos(uint16_t pos);
    uint16_t GetPos() const;
    uint8_t GetID() const;
    void CtrPos(uint16_t pos);
    uint16_t GetCtrPos() const;
    void CtrTime( uint16_t set_time );
    uint16_t GetCtrTime() const;

private:
    uint8_t  ID;
    uint16_t ControlPos;
    uint16_t ControlTime;
    uint16_t pos;
};

class DataPackage{
public: 
    //DataPackage(std::string port, unsigned long baud, uint32_t timeout);
    DataPackage();
    void SendControlPackage( serial::Serial &s);
//    void DecoderPackage(auxiliary ::arm (&p1)[6]);
    //void GetArmPos(uint8_t num);           
    //void GetHeight();                      
    //void GetGimbal();                      
    void CtrGripper(); //modify datapackage gripper control bit
    uint16_t ReturnHeight() const;
    //std::vector<float> ReturnGimbal();
    void ClearCtrPackage();
    //void ClearFeedbackBit();
    void GroupFrames(const auxiliary ::arm (&p1)[6]);
    //void CtrGimbalYaw(uint16_t yaw);
    //void CtrGimbalPitch(uint16_t pitch);
    //uint16_t GetGimbalYaw() const;
    //uint16_t GetGimbalPitch() const;
    void ReceiveMsg(serial::Serial &mySerial, auxiliary::arm (&p1)[6]);
    //void DebugPrint(const auxiliary ::arm (&p1)[6]) const;   
    serial::Serial myserial;
    void SetGripperSta(GripperStatus sta);

private:
    uint8_t package[25];
    uint8_t * rev;
    uint16_t  RevSize; //From one (not zero) to n 
    //uint8_t feedback;
    uint16_t Height;
    //uint16_t CtrYaw;
    //uint16_t CtrPitch;
    //float GimbalPitch; 
    //float GimbalYaw;   
    GripperStatus sta;
};

class auxiliaryNode{
public:
    auxiliaryNode(ros::NodeHandle &n);
    DataPackage myPackage;
    arm myArm[6];
    void InitSubcribers(ros::NodeHandle &n);
    void InitPublishers(ros::NodeHandle &n);
    void InitDataPackageThread();
    void InitOptFlowThread();
    void Publish();
    void DataPackageThread();
    void OpticalFlowThread(); 
    void SetPublishFrequency(uint16_t frequency);


    //void test();
    //void SetTest(int test);
private:
    ros::Publisher AuxiliaryPublisher;
    ros::Publisher OptiFlowPublisher;

    ros::Subscriber ArmControlSubscriber;
    ros::Subscriber GripperControlSubscriber;
    void GetArmControlsCallBack(const auxiliary::controls::ConstPtr& msg);
    void GripperControlsCallBack(const auxiliary::gripper::ConstPtr& msg);
    uint16_t PublishFrequency;
    
    //std::vector<std::_Bind_helper<false, void (auxiliaryNode::*)(auxiliaryNode *),auxiliaryNode*, auxiliaryNode*&>::type> b;
    //std::thread * PublishThread;
};



}


#endif





