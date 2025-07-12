#ifndef MISSION_STATUS_H
#define MISSION_STATUS_H


void safeSigintHandler(int sig);


enum class MissionState
{
    kArm,
    kPrearm,
    kPose,
    kWaybackHome
};


#endif