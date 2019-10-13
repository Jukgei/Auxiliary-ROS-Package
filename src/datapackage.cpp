#include <iostream>
#include <serial.h>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include "../include/auxiliary/auxiliary.hpp"

using namespace std;

//void auxiliary ::DataPackage::GetArmPos(uint8_t num){
//    auxiliary ::DataPackage::feedback &= (1<<num);
//}
//
//void auxiliary ::DataPackage::GetHeight(){
//    auxiliary ::DataPackage::feedback &= 0x01;
//}
//
//void auxiliary ::DataPackage::GetGimbal(){
//    auxiliary ::DataPackage::feedback &= (1<<6);
//}

uint16_t auxiliary ::DataPackage::ReturnHeight() const{
    return (this->Height);
}

//std::vector<float> auxiliary ::DataPackage::ReturnGimbal(){
//    vector<float> ans(2);
//    ans[0] = GimbalYaw;
//    ans[1] = GimbalPitch;
//    return ans;
//}

void auxiliary ::DataPackage::CtrGripper(){
    if(sta == loose)
        this->package[GRIPPER_CTR] = 0x11; 
    else if(sta == grasp)
       package[GRIPPER_CTR] = 0x22; 
    else    //stop 
        package[GRIPPER_CTR] = 0x33;
}

void auxiliary::DataPackage::SetGripperSta(GripperStatus sta){
    this->sta = sta;
}

void auxiliary ::DataPackage::ClearCtrPackage(){
    for(int i = 0; i < 25; i++)
        package[i] = 0;
}

//void auxiliary ::DataPackage::ClearFeedbackBit(){
//    package[FB_CTR] = 0;
//}

//void auxiliary ::DataPackage::CtrGimbalYaw(uint16_t yaw){
//    CtrYaw = yaw;
//}
//
//void auxiliary ::DataPackage::CtrGimbalPitch(uint16_t pitch){
//    CtrPitch = pitch;
//}

//uint16_t auxiliary ::DataPackage::GetGimbalYaw() const{
//    return (this->CtrYaw);
//}
//
//uint16_t auxiliary ::DataPackage::GetGimbalPitch() const{
//    return (this->CtrPitch);
//}

void auxiliary ::DataPackage::SendControlPackage( serial::Serial &s ){
    
    if(s.isOpen()){
        size_t byte = s.write( package, 25 );
        //DEBUG PRINT
        //for(int i = 0; i < 25; i++)
        //    printf("%x ", this->package[i]);
        //printf("\n");
        if(byte == 25){
            //std::cout<<"Send success"<< std::endl;
        }   
        else{
            std::cout<<"It's send "<< byte << " only."<<std::endl;
        }
    }
    else
        std::cout<<"Usart isn't opening."<<endl;
    
}

//void auxiliary ::DataPackage::DecoderPackage(auxiliary ::arm (&p1)[6]){
//    uint8_t i = 0;
//    while((i+1) < RevSize - 1 ){
//        if(rev[i] == rev[i+1]){
//            //Feedback height
//            if(rev[i] == 0xEE && (i+4)<= RevSize){
//                Height = ((uint16_t)rev[i+3])<<8 | rev[i+2];
//                i += 4;
//            }
//            //Feedback gimbal angle
//            //else if(rev[i] == 0xAA && (i+10)<=RevSize){
//            //    uint8_t * buf = new uint8_t[4];
//            //    buf[0] = rev[i+5];
//            //    buf[1] = rev[i+4];
//            //    buf[2] = rev[i+3];
//            //    buf[3] = rev[i+2];
//            //    float * temp = (float *)buf;
//            //    GimbalYaw = *temp;
//            //    i+=4;
//            //    buf[0] = rev[i+5];
//            //    buf[1] = rev[i+4];
//            //    buf[2] = rev[i+3];
//            //    buf[3] = rev[i+2];
//            //    temp = (float *)buf;
//            //    GimbalPitch = *temp;
//            //    i+=10;
//            //    delete []buf;
//            //}
//            //Feedback arm 1 ~ 5 pos
//            else if((0xFF - rev[i]) >= 0x01 && (0xFF - rev[i]) <= 0x05 && (i+4)<=RevSize){
//                uint8_t ID = 0xFF - rev[i];
//                uint16_t pos = ((uint16_t)rev[i+3])<<8 | rev[i+2];
//                p1[ID].SetPos(pos);
//                i += 4;
//            }
//            else{
//                std::cout<<"Data Head error (head not define)"<<std::endl;
//            }
//
//        } 
//        else{
//            std::cout<<"Data Head error (First not equal second)"<<std::endl;
//            std::cout<<"DataPackage is:"<<std::endl;
//            for(int j = 0; j < RevSize; j++)
//                printf("%x ",rev[j]);
//                //std::cout<<rev[j]<<' ';
//            std::cout<<'\n';
//            break;
//        }
//    }
//    delete []rev;
//}

//Should I combine the ReceiveMsg and DecoderPackage? Because the new and delete at two difference function.
void auxiliary ::DataPackage::ReceiveMsg(serial::Serial &mySerial , auxiliary::arm (&p1)[6]){
    //std::cout<<"Coming ReceiveMsg"<<std::endl;
    if(mySerial.isOpen() == true){
        size_t num = mySerial.available();
        this->rev = new uint8_t[num];
        mySerial.read(rev,num);
        RevSize = num;
        //cout<<"Receive num:"<<num<<endl;
    }
    else{
        std::cout<<"Serial is not opening"<<std::endl;
        return ;
    }

    uint8_t i = 0;
    while((i+1) < RevSize - 1 ){
        if(rev[i] == rev[i+1]){
            //Feedback height
            if(rev[i] == 0xEE && (i+4)<= RevSize){
                Height = ((uint16_t)rev[i+3])<<8 | rev[i+2];
                i += 4;
            }
            //Feedback arm 1 ~ 5 pos
            else if((0xFF - rev[i]) >= 0x01 && (0xFF - rev[i]) <= 0x05 && (i+4)<=RevSize){
                uint8_t ID = 0xFF - rev[i];
                uint16_t pos = ((uint16_t)rev[i+3])<<8 | rev[i+2];
                if(pos <= 1000)
                    p1[ID].SetPos(pos);
                i += 4;
            }
            else{
                std::cout<<"Data Head error (head not define)"<<std::endl;
            }

        } 
        else{
            std::cout<<"Data Head error (First not equal second)"<<std::endl;
            printf("First head: %x, Seconde head: %x\n",rev[i],rev[i+1]);
            std::cout<<"DataPackage is:"<<std::endl;
            for(int j = 0; j < RevSize; j++)
                printf("%x ",rev[j]);
                //std::cout<<rev[j]<<' ';
            std::cout<<'\n';
            break;
        }
    }
    delete []rev;
}

auxiliary ::DataPackage::DataPackage(){
    for(uint8_t i = 0;  i < 25 ; i++)
        package[i] = 0;
    rev = nullptr;
    Height = 0; 
    sta = stop;
    //feedback = 0;
    //CtrYaw = 0;
    //CtrPitch = 0;
    //GimbalPitch = 0;
    //GimbalYaw = 0;
    sta = stop;
}

void auxiliary ::DataPackage::GroupFrames(const auxiliary ::arm (&p1)[6]){
    uint8_t package_header = 0xDD;
    package[FIRST_HEADER] = package[SECOND_HEADER] = package_header;
    package[FIRST_TAIL]   = package[SECOND_TAIL] = ~package_header;
    for(int i = 1; i <= 5; i++){
        uint16_t pos    = p1[i].GetCtrPos();
        uint16_t times  = p1[i].GetCtrTime();
        uint8_t index = 4 * i - 2;
        package[index] = (uint8_t)pos;
        package[index+1] = (uint8_t)(pos>>8);
        package[index+2] = (uint8_t)times;
        package[index+3] = (uint8_t)(times>>8);
    }
    CtrGripper();
    //uint16_t yaw    = GetGimbalYaw();
    //uint16_t pitch  = GetGimbalPitch();
    //package[GIMBAL_YAW_CTR_L]   = 0;
    //package[GIMBAL_YAW_CTR_H]   = 0;
    //package[GIMBAL_PITCH_CTR_L] = 0;
    //package[GIMBAL_PITCH_CTR_H] = 0;
    //package[FB_CTR] = feedback;
    
    //for(int i = 1; i <= 5; i ++)
    //        printf("Arm[%d] Ctr Pos: %d, Ctr Time:%d \n",i,p1[i].GetCtrPos(),p1[i].GetCtrTime());
}

//void auxiliary ::DataPackage::DebugPrint( const auxiliary ::arm (&p1)[6] ) const{
//    std::cout<<"======================================"<<std::endl;
//    std::cout<<"Height:"<<Height<<std::endl;
//    //std::cout<<"GimbalYaw:"<<GimbalYaw<<' '<<"GimbalPitch:"<<GimbalPitch<<std::endl;
//    for(uint8_t i = 1; i <= 5; i++){
//        std::cout<<"Arm ID:"<<p1[i].GetID()<<' '<<"Arm Pos:"<<p1[i].GetPos()<<std::endl;
//    }
//    std::cout<<"======================================"<<std::endl;
//}

