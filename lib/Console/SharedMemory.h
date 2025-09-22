#ifndef SHAREDMEMORY_H
#define SHAREDMEMORY_H
#include "CAMEL_SDK/SharedMemory.hpp"

#define ManiControlAL 4

typedef struct _CAMEL_SHM_
{
    bool bIsConnect = false;
    int LanComm_Mode; // TODO: mode???? connection PC; robot(3), local(1) ... etc
    int mc_ch; // TODO: last selected???

    LAN_CAMEL2GUI CAMEL_DATA_NEW;

    COMMAND_STRUCT  COMMAND;
    bool NEWCOMMAND;
    ROS2_DATA ros2Data;
}CAMEL_SHM, *pCAMEL_SHM;
extern pCAMEL_SHM sharedCamel;

#endif // SHAREDMEMORY_H
