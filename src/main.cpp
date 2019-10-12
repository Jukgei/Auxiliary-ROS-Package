#include <iostream>
#include "../include/auxiliary/auxiliary.hpp"
#include <ros/ros.h>
#include <stdlib.h>
#include "auxiliary/state.h"
#include "control/controls.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "auxiliary");
    ros::NodeHandle n;
    auxiliary::auxiliaryNode * AuxiliaryNode = new auxiliary::auxiliaryNode(n);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    delete AuxiliaryNode;
    AuxiliaryNode = nullptr;

    return 0;
}

