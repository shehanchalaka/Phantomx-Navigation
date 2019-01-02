#include <ros/ros.h>

#include "phantomx_control/phantomx_control.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "phantomx_control_node");
    ros::NodeHandle nodeHandle("~");

    PhantomxControl phantomxControl(nodeHandle);
    ros::spin();

    return 0;
}