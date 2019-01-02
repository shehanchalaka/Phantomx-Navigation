#include <ros/ros.h>
#include "phantomx_state_estimator/state_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "phantomx_state_extimator");
    ros::NodeHandle nodeHandle("~");

    StateEstimator stateEstimator(nodeHandle);

    ros::spin();

    return 0;
}