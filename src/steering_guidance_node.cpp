#include "steering_guidance_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_guidance");

    ros::NodeHandle nh("~");

    SteeringGuidance core(nh);
    return 0;
}

