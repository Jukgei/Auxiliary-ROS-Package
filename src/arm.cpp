#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include "../include/auxiliary/arm.hpp"
//#include "../include/auxiliary/auxiliary.hpp"

using namespace std;
/*
auxiliary ::arm::arm(uint8_t num){
    this->ID = num;
} */
void auxiliary ::arm::SetID(uint8_t ID){
    this->ID = ID;
}

void auxiliary ::arm::SetPos(uint16_t pos){
    this->pos = pos;
}

uint16_t auxiliary ::arm::GetPos() const{
    return (this-> pos);
}

uint8_t auxiliary ::arm::GetID() const{
    return (this->ID);
}

void auxiliary ::arm::CtrPos(uint16_t pos){
    this->ControlPos = pos;
}

uint16_t auxiliary ::arm::GetCtrPos() const{
    return (this->ControlPos);
}

void auxiliary ::arm::CtrTime(uint16_t set_time){
    this->ControlTime = set_time;
}

uint16_t auxiliary ::arm::GetCtrTime() const{
    return (this->ControlTime); 
}
