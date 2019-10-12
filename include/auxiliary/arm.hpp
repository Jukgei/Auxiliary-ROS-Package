#ifndef ARM_H
#define ARM_H

#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
//#include "../include/auxiliary/auxiliary.hpp"

namespace auxiliary{
    
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

}


#endif
