//
// Created by ys on 24. 2. 16.
//

#ifndef RBCANINE_SHAREDMEMORY_HPP
#define RBCANINE_SHAREDMEMORY_HPP

#include <Eigen/Dense>
#include <Eigen/Core>
#include "ENumClasses.hpp"
#include <heightmaptools/HeightmapTools.hpp>
#include <heightmapcore/HeightmapCore.hpp>

#define CAMEL_SHM_NAME_CORE      "CAMEL_CORE_SHARED_MEMORY_ROBOT_DATA"


#define PI          3.141592
#define R2D         57.2957802
#define D2R         0.0174533

#define MAX_PODO_NUM 10

#define RT_MS       2
#define dT 0.002
#define MAX_JOINT 12
#define MAX_LEG 4

#define CMD_dT              0.002
#define HIGH_CONTROL_dT     0.01
#define LOW_CONTROL_dT      0.002
#define SHORT_PREDICTION_dT      0.02
#define LONG_PREDICTION_dT      0.1
#define LOW_CONTROL_FREQ    500
#define VISUAL_dT           0.002
#define ESTIMATOR_dT        0.002
#define MAX_COMMAND_DATA        40
#define MPC_HORIZON         8

/// --------------------------- Struct Information ---------------

typedef struct _GAMEPAD_
{
    float L_UD = 0.0f, L_RL = 0.0f, R_UD = 0.0f, R_RL = 0.0f;
    float LT = 0.0f, RT = 0.0f;
    bool BTN[16] = {false,};
    bool status = false;
} GAMEPAD, *pGAMEPAD;

typedef struct __LAN_STRUCT_JOYSTICK__
{
    float axisLeftX = 0.0f;
    float axisLeftY = 0.0f;

    float axisRightX = 0.0f;
    float axisRightY = 0.0f;

    float triggerLeft = 0.0f;
    float triggerRight = 0.0f;

    unsigned char buttons[16] = {0,};
} LAN_JOYSTICK;

typedef struct __LAN_STRUCT_MIDDLEWARE__
{
    float linearVelocityX = 0.0f;
    float linearVelocityY = 0.0f;
    float angularVelocityZ = 0.0f;
} LAN_MIDDLEWARE;

typedef struct _COMMAND_STRUCT_
{
    int     COMMAND_TARGET = 0;
    int     USER_COMMAND = 0;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA] = {0,};
    int	    USER_PARA_INT[MAX_COMMAND_DATA] = {0,};
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA] = {0.0f,};
    double  USER_PARA_DOUBLE[MAX_COMMAND_DATA] = {0.0,};
} COMMAND_STRUCT, *pCOMMAND_STRUCT;

typedef struct _ROBOT_RAW_DATA_
{
    struct
    {
        double position[MAX_JOINT];
        double velocity[MAX_JOINT];
        double torque[MAX_JOINT];
        double temperature[MAX_JOINT];
        double voltage[MAX_JOINT];
        double Kp[MAX_JOINT];
        double Kd[MAX_JOINT];
        int error_code[MAX_JOINT];
    } motor;

    struct
    {
        Eigen::Quaterniond quat;
        Eigen::Vector3d rpy;
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;
    } IMU;

    GAMEPAD gamepad;

    double battery_voltage;
    bool can_check;
    bool find_home;
    bool control_start;

} ROBOT_RAW_DATA;

typedef struct _MIDDLEWARE_DATA_
{
    int FSMState;
    double localTime;

    struct
    {
        Eigen::Vector3d baseVelocity;
        Eigen::Vector3d baseAngularVelocity;
    } body;

    struct
    {
        Eigen::Vector3d basePosition;
        Eigen::Vector3d baseVelocity;
        Eigen::Vector3d baseAngularVelocity;
        Eigen::Vector3d rpy; // body orientation[rpy]
        Eigen::Vector4d quat; // body orientation[quat]
    } global;


} MIDDLEWARE_DATA;

typedef struct _ROS2_DATA_
{
    int command = 0;
    int prevCommand = 0;
    bool bNewCommand = false;
    float baseRefVelX;
    float baseRefVelY;
    float baseRefAngVelZ;
} ROS2_DATA;

typedef struct _COMMAND_
{
    int     COMMAND_TARGET = 0;
    int     USER_COMMAND = 0;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA] = {0,};
    int	    USER_PARA_INT[MAX_COMMAND_DATA] = {0,};
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA] = {0.0f,};
    double  USER_PARA_DOUBLE[MAX_COMMAND_DATA] = {0.0,};
} COMMAND, *pCOMMAND;

typedef struct _MAP_
{
    heightmaptools::Point3 heightMap[mapspec::LOCAL_SIZE_SQUARED];
    size_t heightMapSize;

    std::array<std::array<Eigen::Vector3d, 20>, 36> elevationMap;
    std::array<std::array<Eigen::Vector3d, 20>, 36> elevationCostMap;
    std::pair<double, double> gridMapLength;
    std::pair<double, double> gridMapCenter;
    std::pair<int, int> gridMapSize;
    double gridMapResolution;

    std::array<std::array<Eigen::Vector3d, 50>, 4> feetCandidates;
    std::array<int, 4> feetCandidatesSize;

    bool isVisionFSM;
    bool heightMapUpdateFlag;
    bool elevationMapUpdateFlag;

    std::array<Eigen::Vector3d, 4> plannedFeet;
} MAP, *pMAP;


typedef struct _CAMEL_SHM_DATA_
{
    int gaitTable[MPC_HORIZON * 4];
    int FSMState;
    double localTime;

    ROBOT_RAW_DATA rawData;
    COMMAND command[5];

    struct _JOINT_
    {
        double position[MAX_JOINT];
        double velocity[MAX_JOINT];
        double torque[MAX_JOINT];
        double errorStatus[MAX_JOINT];
        double desiredPosition[MAX_JOINT];
        double desiredVelocity[MAX_JOINT];
        double desiredTorque[MAX_JOINT];
        double desiredKp[MAX_JOINT];
        double desiredKd[MAX_JOINT];
    } joint;

    struct
    {
        Eigen::Vector3d rpy; // body orientation[rpy]
        Eigen::Vector4d quat; // body orientation[quat]
        Eigen::Vector3d desiredRpy; // body desired orientation[rpy]
        Eigen::Vector4d desiredQuat; // body desired orientation[quat]
    } orientation;

    struct
    {
        Eigen::Vector3d basePosition;
        Eigen::Vector3d baseVelocity;
        Eigen::Vector3d baseDesiredPosition;
        Eigen::Vector3d baseDesiredVelocity;
        Eigen::Vector3d baseDesiredAcceleration;
        Eigen::Vector3d baseAngularVelocity; // body angular velocity
        Eigen::Vector3d baseDesiredAngularVelocity;
    } body;

    struct
    {
        Eigen::Vector3d basePosition;
        Eigen::Vector3d baseVelocity;
        Eigen::Vector3d baseDesiredPosition;
        Eigen::Vector3d baseDesiredVelocity;
        Eigen::Vector3d baseDesiredAcceleration;
        Eigen::Vector3d baseAngularVelocity;
        Eigen::Vector3d baseDesiredAngularVelocity;
        Eigen::Vector3d baseAcceleration; // body acceleration
        Eigen::Vector3d groundCenter;
        Eigen::Vector2d baseCapturePoint;
        Eigen::Vector2d baseZeroMomentPoint;
        Eigen::Vector2d baseDesiredZeroMomentPoint;
        Eigen::Vector2d baseDesiredCP;
        Eigen::Vector2d baseFutureCapturePoint;
        Eigen::Vector2d footIntersection;
        double baseHeight;
    } global;

    struct
    {
        Eigen::Vector3d targetRPY;
        Eigen::Vector3d targetGlobalPos;
        Eigen::Vector3d targetAngularVel;
        Eigen::Vector3d targetGlobalVel;
    } mpc;

    struct
    {
        bool allLegContact;
        bool clearCP;
        bool swingDone;
        bool blockGaitSwitching;
    } flags;

    Eigen::Vector3d pdTorque[4];
    Eigen::Array<double, MAX_JOINT, 1> wbTorque;
    Eigen::Vector3d globalFootPosition[4];
    Eigen::Vector3d globalFootContactPos[4];
    Eigen::Vector3d bodyBase2FootPosition[4];
    Eigen::Vector3d bodyBase2FootVelocity[4];
    Eigen::Vector3d bodyBase2FootDesiredPosition[4];
    Eigen::Vector3d bodyBase2FootDesiredVelocity[4];
    Eigen::Vector3d bodyBase2FootDesiredAcceleration[4];
    Eigen::Vector3d globalBase2FootDesiredAcceleration[4];

    double estimatedTorque[12];
    double contactProbHz[4];
    double contactProbFz[4];
    double contactProbGait[4];
    double contactEstimatedStates[4];
    bool bContactState[4];
    int mContactStack[4];
    double gaitWeightingFactor[4];
    double timeStep[4];
    bool CartesianContactState[4][3];
    bool lateLanding[4];
    Eigen::Vector3d solvedGRF[4];
    Eigen::Vector3d solvedMPCGRF[4];
    Eigen::Vector3d estimatedGRF[4];

    double threadElapsedTime[11];

    // For Motion_Control
    int gaitState;
    double swingPeriod;
    double standPeriod;
    bool bSwingStart[4];
    bool bGaitPhase[MPC_HORIZON * MAX_LEG];
    bool bLastSwingLeg[4];
    bool bIsEndHome;
    double liftOffTime[4];

    struct _GAIT_
    {
        double duty;
        double phase;
    }gait[MAX_LEG];

    Eigen::Vector3d estimatedSlopeRPY;
    Eigen::Matrix3d estimatedSlopeRot;
    Eigen::Vector4d estimatedSlopeCoeff;
    Eigen::Vector3d estimatedExternalForce;
    Eigen::Vector3d estimatedExternalMomentum;
    Eigen::Vector3d relativeRPY;

    double globalFootHeight[4];

    Eigen::Vector3d swingPgain[4];
    Eigen::Vector3d swingDgain[4];

    Eigen::Vector3d mpcDesiredPos[MPC_HORIZON];

    struct
    {
        Eigen::Vector3d LinVelocity;
        Eigen::Vector3d AngVelocity;
        bool bRosCMDReceived = false;
    } rosCommand;

    Eigen::Vector3d commandLinearVelocity;
    Eigen::Vector3d commandAngularVelocity;

    int gaitCommand;
    int contactNum;

    /// divergence
    bool bIsDiv;

    bool bCompliantMode;
    bool bGDMCommand;
    bool bConsoleCommand;
    bool bTurboMode;
    bool bTrotStopping;
    bool bVisionMode;

    // vision
    Eigen::Vector3d globalGoalFootPosition[4];
    Eigen::Vector3d globalTargetFootPosition[4];
    Eigen::Vector3d globalLegTraj[4];

    struct
    {
        Eigen::Vector3d baseDesiredVelocity;
        Eigen::Vector3d baseDesiredAngularVelocity;
    } start;

    double stanceTime;
    Eigen::Vector3d globalGoalBasePosition;
    Eigen::Vector3d globalGoalBaseRPY;
    Eigen::Vector3d globalStartBasePosition;
    Eigen::Vector3d globalStartBaseRPY;
    Eigen::Vector3d globalVisualBasePosition;
    double mapSize;
    double mapForLogger[625][3]; // TODO: height map size
    float mapHeight;
    bool isMapUpdated;
    bool visionAllowed;
    bool isUdpConnected;
    bool isROSConnected;
    Eigen::Vector3d footCandidatesVisual[4][400];
    Eigen::Vector3d stairStartNode;

    MAP vision;
    int test;

    int stairState; // 0: stop, 1: up, 2: down, 3: flat

    Eigen::Vector3d goalPos;
    Eigen::Vector3d startPos;
    Eigen::Vector3d swingEndPos;
    Eigen::Vector3d zmp[10];

} CAMEL_SHM_CORE, *pCAMEL_SHM_DATA;

// ===================================================
// LAN Structure
typedef struct _LAN_STRUCT_CAMEL2GUI_
{
    FSM fsm_state;
    int gait_state;
    bool leg_contact[MAX_LEG];
    bool bGDMCommand;
    bool bConsoleCommand;
    bool bVisionAvailable;
    bool bTurboMode;
    bool bVisionMode;
    Eigen::Vector3d cmd_vel;
    Eigen::Vector3d actual_vel;

    bool isVisionUdpConnected;
    bool isROSConnected;

    double batteryVoltage;

    ROBOT_RAW_DATA rawData;
    double desiredTorque[MAX_JOINT];
    double desiredPosition[MAX_JOINT];

    MIDDLEWARE_DATA middlewareData;
} LAN_CAMEL2GUI, *pLAN_CAMEL2GUI;

#endif //RBCANINE_SHAREDMEMORY_HPP
