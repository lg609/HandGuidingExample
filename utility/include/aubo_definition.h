#ifndef AUBO_DEFINITION_H
#define AUBO_DEFINITION_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
using namespace std;



//for Ruisheng High Speed Arm
#define HIGH_SPEED_ARM

//parameters for control
#define MAX_ACC_MOTOR       20000
#define MAX_SPEED_MOTOR60   3000
#define MAX_SPEED_MOTOR80   3000

// Macro definition of modular reduction ratio
#define GEAR_RATIO_J60		101         /**< Reduction ratio of J60 */
#define GEAR_RATIO_J80		121         /**< Reduction ratio of J80 */

#define MIN_CONTROL_TIMER   5
#define CONTROL_PERIOD      (MIN_CONTROL_TIMER/1000.0)
#define ARM_DOF             6
#define OTGNEW_JERK_ACC_RATIO 8

//parameters for caluculation
#define RPM_TO_RAD_PER_SEC     0.104720    /* 2*PI/60 */
#define RPM_TO_RAD_PER_SEC_STEP (RPM_TO_RAD_PER_SEC*CONTROL_PERIOD)
#define JOINT_MAX_POS       (175.0/180*M_PI)
//#define JOINT_MIN_POS       (-JOINT_MAX_POS)
#define JOINT_RESOLUTION 0.000003062 /*2*Pi/(1060*16*121)*/
#define JOINT_ARRIVAL_RESOLUTION 0.000349 /*0.02 degree*/
#define JOINT_ARRIVAL_THR    0.000873 /*in steady of 0.0012*/
#define HALF_CARTESIAN_RESOLUTION 0.0005
#define TEACH_END_STEP  (0.01*M_PI/180.0) /**5*speed/100*/
#define TEACH_JOINT_STEP  (0.1*M_PI/180.0) /**5*speed/100*/
#define MAX_SPEED_JOINT (MAX_SPEED_MOTOR60/GEAR_RATIO_J60)
#define OVERSPEED_THR_TOTAL 1.2
#define OVERSPEED_THR_SINGLE 1.5

//switches
#define THROW_ROBOT_EXCEPTION(type, error_code, error_msg)
//#define OPTIMAL_JOINT_TRAJ_FORWARD_DYNAMICS
#define BLOCKING_OPERATION


#define MOTION_CONF_ERR_COUNT 12

#define ZERO_THRESH 1e-5
#define PI  M_PI

typedef struct AUBO_ROBOT_DH_Parameters {
    double d1;
    double d2;
    double d5;
    double d6;
    double a2;
    double a3;
}ARDP;

//structures
enum aubo_robot_type
{
    aubo_i5,
    aubo_i7,
    aubo_i10_12,
    aubo_i3s,
    aubo_i3,
    aubo_i5s,
    aubo_i5l,
    aubo_i10s
};

namespace MoveConditionClass
{
    enum robot_state
    {
        RobotStopped = 0,
        RobotRunning,
        RobotPausing,
        RobotPaused,
        RobotResuming //temporary state.
    };

    enum move_mode
    {
        NO_MOVEMODE = 0,
        MODEJ,
        MODEL,
        MODEP
    };

    enum move_track
    {
        NO_TRACK = 0,

        //for moveJ and moveL
        TRACKING,

        //cartesian motion for moveP
        ARC_CIR,
        CARTESIAN_MOVEP,
        CARTESIAN_CUBICSPLINE,
        CARTESIAN_UBSPLINEINTP,
        CARTESIAN_GNUBSPLINEINTP,
        CARTESIAN_LOOKAHEAD,


        //joint motion for moveP
        JIONT_CUBICSPLINE,        
        JOINT_UBSPLINEINTP,
        JOINT_GNUBSPLINEINTP,
    };

    enum coordinate_refer
    {
        BaseCoordinate = 0,
        EndCoordinate,
        WorldCoordinate
    };

    enum teach_mode
    {
        NO_TEACH = 0,
        JOINT1,
        JOINT2,
        JOINT3,
        JOINT4,
        JOINT5,
        JOINT6,
        MOV_X,
        MOV_Y,
        MOV_Z,
        ROT_X,
        ROT_Y,
        ROT_Z
    };

    enum arrival_ahead_state
    {
        arrival_ahead_none,
        arrival_ahead_distance, //in meter
        arrival_ahead_time, //in second
        arrival_ahead_blend_distance,
        arrival_ahead_blend_time,
        arrival_ahead_done //sent arrival signal for tracking but block arrival signal for blending.
    };
} //end of MoveConditionClass

//waypoint.
struct Pos
{
    double x;
    double y; 
    double z; 
};

union cartesianPos_U
{
    Pos position;
    double positionVector[3];
};

struct Ori
{
    double w; 
    double x; 
    double y;
    double z; 
};


union cartesianOri_U
{
    Ori orientation;
    double quaternionVector[4];
};

typedef struct
{
    cartesianPos_U cartPos;
    //cartesianOri_U cartOri;
    Ori orientation;
    //double eulerAngle[3];
    double jointpos[ARM_DOF];
}wayPoint_S;
#define RoadPoint wayPoint_S

typedef struct
{
    double jointPos[ARM_DOF];
}JointParam;



/** 坐标系标定方法枚举 **/
enum CoordCalibrateMathod
{
    Origin_AnyPointOnPositiveXAxis_AnyPointOnPositiveYAxis,            // 原点、x轴正半轴、y轴正半轴
    Origin_AnyPointOnPositiveYAxis_AnyPointOnPositiveZAxis,            // 原点、y轴正半轴、z轴正半轴
    Origin_AnyPointOnPositiveZAxis_AnyPointOnPositiveXAxis,            // 原点、z轴正半轴、x轴正半轴
    Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane,  // 原点、x轴正半轴、x、y轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOZPlane,  // 原点、x轴正半轴、x、z轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOZPlane,  // 原点、y轴正半轴、y、z轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOXPlane,  // 原点、y轴正半轴、y、x轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOXPlane,  // 原点、z轴正半轴、z、x轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOYPlane,  // 原点、z轴正半轴、z、y轴平面的第一象限上任意一点

    CoordTypeCount
};

/** 该结构体描述工具的参数
  * 工具标定后既有相对末端坐标系的位置也有姿态。
  */
typedef struct
{
    Pos        toolInEndPosition;     //工具相对末端坐标系的位置

    Ori        toolInEndOrientation;  //工具相对末端坐标系的姿态
}ToolInEndDesc;

/** 坐标系标定 **/
typedef struct
{
    MoveConditionClass::coordinate_refer     coordType;

    CoordCalibrateMathod methods;           //标定方法

    wayPoint_S           wayPointArray[3];  //用于标定坐标系的３个点  对应于机械臂指的是工具末端的点

    ToolInEndDesc     toolInEnd;

}CoordCalibrateByToolEndPoint;


/**
 * 该结构体用户描述一个坐标系。系统根据该结构体能够唯一个确定一个坐标系。
 *
 * 坐标系分３种类型：基座标系(BaseCoordinate)，末端坐标系(EndCoordinate)，用户坐标系(WorldCoordinate);
 *
 *　基座坐标系是　根据机械臂底座建立的坐标系
 *　末端坐标系是　根据机械臂末端建立的坐标系
 *　用户坐标系是　用户根据自己的需求建立的实际需要用到的坐标系,系统根据用户提供的３个点和标定方法确定用户坐标系的X轴,Y轴,Z轴。
 * 　　　　　　　 在实际应用中标定坐标系时会用到工具，系统为了准确的得到坐标系标定的３个点，所以用户需要提供工具信息，
 * 　　　　　　　 如果没有使用工具可以将工具描述(toolDesc)设置为０．
 *
 * 使用说明：
 * 　　1:当coordType==BaseCoordinate时，下面3个参数(methods,wayPointArray,toolDesc)系统不做处理，
 * 　　　当coordType==EndCoordinate，表示末端坐标系，其中末端的工具参数通过toolDesc给出
 * 　　　当coordType==WorldCoordinate，表示用户坐标系，坐标系的描述通过CoordCalibrateMathod(标定方法),JointParam(3个点),ToolInEndDesc(标定时使用的工具)给出
 *
 * 　　2:如果坐标系标定的时候没有使用工具，工具描述中的位置信息为(0,0,0)和姿态信息应设置为(1,0,0,0)。
 */
typedef struct
{
    MoveConditionClass::coordinate_refer    coordType;
    /**
     * 坐标系类型：基座坐标系　　　　当coordType==BaseCoordinate
                 末端坐标系　　　　当coordType==EndCoordinate，表示末端坐标系，其中末端的工具参数通过toolDesc给出
                 用户自定义坐标系　当coordType==WorldCoordinate，表示用户坐标系，坐标系的描述通过CoordCalibrateMathod(标定方法),JointParam(3个点),ToolInEndDesc(标定时使用的工具)给出
     **/

    CoordCalibrateMathod methods;        // 坐标系标定方法

    JointParam       wayPointArray[3];   //用于标定坐标系的３个点（关节角），对应于机械臂法兰盘中心点基于基座标系

    ToolInEndDesc    toolDesc;           //该参数为复用参数　　当coordType==EndCoordinate时，表示末端的工具参数　  当coordType==WorldCoordinate，表示标定用户坐标系的时使用的工具

}CoordCalibrateByJointAngleAndTool;



typedef ToolInEndDesc ToolKinematicsParam;

/** 该结构体描述工具惯量　**/
typedef struct
{
    double xx;
    double xy;
    double xz;
    double yy;
    double yz;
    double zz;
}ToolInertia;

/**
 * 该结构体描述工具的　动力学参数
 *
 * 注意：在跟换机械臂的工具时，工具的动力学参数和运动学参数是需要一起设置的。
 **/
typedef struct
{
    double positionX;    //工具重心的X坐标

    double positionY;    //工具重心的Y坐标

    double positionZ;    //工具重心的Z坐标

    double payload;      //工具重量

    ToolInertia toolInertia;  //工具惯量

}ToolDynamicsParam;

//interface.
typedef union
{
    double cartPara[2];
    double jointPara[ARM_DOF];
}joint_cart_U;

typedef struct 
{
    bool objectConnected;
    char movej_slowStop_downGrasp; //no used by now
    double downGrapDistance;
    double conveyorSpeed;
    double conveyorRotInBase[9]; //unit vector of conveyor x-axis in base coordinate
    //double objectCurrPositionInBase[3]; //object positon in base coordinage
    double workAreaDownLimit; //distance from robot origin(current position) to work area boundary along conveyor x-axis for high layer(conveyor chasing).
    double timingAdjust;    
}ConveyorProfile;

typedef struct
{
    cartesianPos_U posU;
    cartesianOri_U oriU;
}Pose_S;

typedef struct  
{
    MoveConditionClass::move_mode moveMode;
    MoveConditionClass::move_track subMoveMode;
    MoveConditionClass::teach_mode teachMode;
    bool enableIterIk;//是否可以使用迭代近似逆解
    bool toolTrack;
    cartesianPos_U toolInEndPosition;
    Ori toolInEndOrientation;
    // conveyor tracking(trajectory) stop(pause)/blending in new svn branch.
    ConveyorProfile conveyorCfg;

    struct
    {
        bool ena;
        float relativePosition[3];
        cartesianOri_U relativeOrientation; // relative orientation.
    }relativeMove;

    //joint(no motor): radian/s radian/s2
    //cartesian(no motor): m/s m/s2
    joint_cart_U maxVelc;
    joint_cart_U maxAcc;

    struct
    {
        MoveConditionClass::arrival_ahead_state arrivalAheadStat;
        double arrivalAheadThr;   //米或者秒
    }arrivalAhead;
    
    float blendRadius;
    int circularLoopTimes;
    double otgJerkAccRatio;

}RobotMoveProfile;

//namespace MoveConditionClass
//{
//void motionProfileInit(RobotMoveProfile & mp);
//bool motionProfileSetup(RobotMoveProfile & mp, double *velcPtr,
//                        double *accPtr, MoveConditionClass::move_mode mode,
//                        MoveConditionClass::move_track moveTrackType,
//                        bool endMotion, MoveConditionClass::teach_mode teach = MoveConditionClass::NO_TEACH, bool iterIk = false);
//}

typedef struct
{
    double minValue;
    double maxValue;

}RangeOfMotion;

/**
 * 关节运动范围
 */

typedef struct
{
    bool   enable;                        // 是否使能偏移

    RangeOfMotion rangeValues[ARM_DOF];    //运动范围

}JointRangeOfMotion;

typedef struct
{
    bool enable;
    wayPoint_S wps[4]; //origin first + 3 neighbours
}cuboidRangeWithTool;

typedef struct
{
    bool enable;
    double baseToRot[9];
    double baseToPos[3];
    double maxXYZ[3];
}cartRangeOfMotion;
/**
 * event define   机械臂事件类型
 */
typedef enum{

    RobotEvent_armCanbusError,
    RobotEvent_remoteHalt,
    RobotEvent_remoteEmergencyStop,
    RobotEvent_jointError,               //关节错误

    RobotEvent_forceControl,             //力控制
    RobotEvent_exitForceControl,         //退出力控制

    RobotEvent_softEmergency,            //软急停
    RobotEvent_exitSoftEmergency,        //软急停


    RobotEvent_collision,                //碰撞
    RobotEvent_collisionStatusChanged,   //碰撞

    RobotEvent_tcpParametersSucc,

    RobotEvent_powerChanged,             //机械臂电源开关状态改变
    RobotEvent_ArmPowerOff,

    RobotEvent_mountingPoseChanged,     //安装位置发生改变
    RobotEvent_encoderError,            //编码器错误

    RobotEvent_encoderLinesError,      //编码器线数不一致
    RobotEvent_singularityOverspeed,   //奇异点超速
    RobotEvent_currentAlarm,           //机械臂电流异常


    RobotEvent_toolioError,           //机械臂工具端错误

    RobotEvent_robotStartupPhase,
    RobotEvent_robotStartupDoneResult,
    RobotEvent_robotShutdownDone,
    RobotEvent_atTrackTargetPos,      //轨迹到位信号


    RobotSetPowerOnDone,
    RobotReleaseBrakeDone,

    RobotEvent_robotControllerStateChaned,
    RobotEvent_robotControllerError,
    RobotEvent_socketDisconnected,
    RobotEvent_robotControlException,
    RobotEvent_trackPlayInterrupte,

    RobotEvent_staticCollisionStatusChanged,
    RobotEvent_MountingPoseWarning,
    RobotEvent_MacDataInterruptWarning,
    RobotEvent_ToolIoError,
    RobotEvent_InterfacBoardSafeIoEvent,


    RobotEvent_RobotHandShakeSucc,
    RobotEvent_RobotHandShakeFailed,

    RobotEvent_RobotErrorInfoNotify,

    RobotEvent_InterfacBoardDIChanged,
    RobotEvent_InterfacBoardDOChanged,
    RobotEvent_InterfacBoardAIChanged,
    RobotEvent_InterfacBoardAOChanged,

    RobotEvent_UpdateJoint6Rot360Flag,

    RobotEvent_RobotMoveControlDone,
    RobotEvent_RobotMoveControlStopDone,
    RobotEvent_RobotMoveControlPauseDone,
    RobotEvent_RobotMoveControlContinueDone,

    //主从模式切换
    RobotEvent_RobotSwitchToOnlineMaster,
    RobotEvent_RobotSwitchToOnlineSlave,


    RobotEvent_ConveyorTrackRobotStartup,
    RobotEvent_ConveyorTrackRobotCatchup,


    RobotEvent_overSpeed,                           //超速
    RobotEvent_algorithmException,                  //机械臂算法异常

    //interface board io event
    RobotEvent_boardIoPoweron,                     //外部上电信号
    RobotEvent_boardIoRunmode,                     //联动/手动
    RobotEvent_boardIoPause,                       //外部暂停信号
    RobotEvent_boardIoStop,                        //外部停止信号
    RobotEvent_boardIoHalt,                        //外部关机信号
    RobotEvent_boardIoEmergency,                   //外部急停信号
    RobotEvent_boardIoRelease_alarm,               //外部报警解除信号
    RobotEvent_boardIoOrigin_pose,                 //外部回原点信号
    RobotEvent_boardIoAutorun,                     //外部自动运行信号

    //interface board safety io event
    RobotEvent_safetyIoExternalEmergencyStope,   //外部急停输入01
    RobotEvent_safetyIoExternalSafeguardStope,   //外部保护停止输入02
    RobotEvent_safetyIoReduced_mode,               //缩减模式输入
    RobotEvent_safetyIoSafeguard_reset,            //防护重置
    RobotEvent_safetyIo3PositionSwitch,           //三态开关1
    RobotEvent_safetyIoOperationalMode,           //操作模式
    RobotEvent_safetyIoManualEmergencyStop,      //示教器急停01
    RobotEvent_safetyIoSystemStop,                //系统停止输入

    //robot working event
    RobotEvent_alreadySuspended,                    //机械臂暂停
    RobotEvent_alreadyStopped,                      //机械臂停止
    RobotEvent_alreadyRunning,                      //机械臂运行

    RobotEvent_exceptEvent = 100,

    RobotEventUndefined                         =1000,   // 未定义事件



    /**
     * RobotControllerErrorEvent  控制器异常事件 1001~1499
     *
     * 事件处理建议
     * 建议采取措施:暂停当前运动
     *
     * PS: 这些事件会引起机械臂运动的错误返回
     *     使用时尽量用枚举变量　　枚举变量值只是为了查看日志方便
     *
     **/
    RobotEventMoveJConfigError                 = 1001,  // moveJ configuration error 关节运动属性配置错误
    RobotEventMoveLConfigError                 = 1002,  // moveL configuration error 直线运动属性配置错误
    RobotEventMovePConfigError                 = 1003,  // moveP configuration error 轨迹运动属性配置错误
    RobotEventInvailConfigError                = 1004,  // invail configuration      无效的运动属性配置
    RobotEventWaitRobotStopped                 = 1005,  // please wait robot stopped 等待机器人停止
    RobotEventJointOutRange                    = 1006,  // joint out of range        超出关节运动范围
    RobotEventFirstWaypointSetError            = 1007,  // please set first waypoint correctly in modep    请正确设置MODEP第一个路点
    RobotEventConveyorTrackConfigError         = 1008,  // configuration error for conveyor tracking       传送带跟踪配置错误
    RobotEventConveyorTrackTrajectoryTypeError = 1009,  // unsupported conveyor tracking trajectory type   传送带轨迹类型错误
    RobotEventRelativeTransformIKFailed        = 1010,  // inverse kinematics failure due to invalid relative transform  相对坐标变换逆解失败
    RobotEventTeachModeCollision               = 1011,  // collision in teach-mode  示教模式发生碰撞
    RobotEventextErnalToolConfigError          = 1012,  // configuration error for external tool and hand workobject     运动属性配置错误,外部工具或手持工件配置错误

    RobotEventTrajectoryAbnormal               = 1101,  // Trajectory is abnormal 轨迹异常
    RobotEventOnlineTrajectoryPlanError        = 1102,  // Trajectory is abnormal,online planning failed  轨迹规划错误
    RobotEventOnlineTrajectoryTypeIIError      = 1103,  // Trajectory is abnormal,type II online planning failed 二型在线轨迹规划失败
    RobotEventIKFailed                         = 1104,  // Trajectory is abnormal,inverse kinematics failed 逆解失败
    RobotEventAbnormalLimitProtect             = 1105,  // Trajectory is abnormal,abnormal limit protection 动力学限制保护
    RobotEventConveyorTrackingFailed           = 1106,  // Trajectory is abnormal,conveyor tracking failed  传送带跟踪失败
    RobotEventConveyorOutWorkingRange          = 1107,  // Trajectory is abnormal,exceeding the conveyor working range 超出传送带工作范围
    RobotEventTrajectoryJointOutOfRange        = 1108,  // Trajectory is abnormal,joint out of range 关节超出范围
    RobotEventTrajectoryJointOverspeed         = 1109,  // Trajectory is abnormal,joint overspeed 关节超速
    RobotEventOfflineTrajectoryPlanFailed      = 1110,  // Trajectory is abnormal,Offline track planning failed 离线轨迹规划失败

    RobotEventControllerIKFailed               = 1200,  // The controller has an exception and the inverse kinematics failed 控制器异常，逆解失败
    RobotEventControllerStatusException        = 1201,  // The controller has an exception and the status is abnormal 控制器异常，状态异常
    RobotEventControllerTrackingLost           = 1202,  // Exception that joint tracking is lost, 关节跟踪误差过大.

    RobotEventMoveEnterStopState               = 1300,  // Movement enters the stop state 运动进入到stop阶段

    RobotEventControllerAbnormalMaxIndex       = 1999,  //

    /**
     * RobotHardwareErrorEvent  控制器异常事件 2001~2999
     *
     * 事件处理建议
     * RobotEventJointEncoderPollustion   建议采取措施:警告性通知
     * RobotEventJointCollision           建议采取措施:暂停当前运动  一遍恢复运动
     *
     * 其余的事件　　建议采取措施:暂停当前运动
     **/

    RobotEventHardwareErrorNotify              = 2001,  // Robot hardware error 机械臂硬件错误

    RobotEventJointError                       = 2101,  // Robot joint error 机械臂关节错误
    RobotEventJointOverCurrent                 = 2102,  // Robot joint over current.  机械臂关节过流
    RobotEventJointOverVoltage                 = 2103,  // Robot joint over voltage.　 机械臂关节过压
    RobotEventJointLowVoltage                  = 2104,  // Robot joint low voltage.　  机械臂关节欠压
    RobotEventJointOverTemperature             = 2105,  // Robot joint over temperature. 机械臂关节过温
    RobotEventJointHallError                   = 2106,  // Robot joint hall error. 机械臂关节霍尔错误
    RobotEventJointEncoderError                = 2107,  // Robot joint encoder error. 机械臂关节编码器错误
    RobotEventJointAbsoluteEncoderError        = 2108,  // Robot joint absolute encoder error. 机械臂关节绝对编码器错误
    RobotEventJointCurrentDetectError          = 2109,  // Robot joint current position error. 机械臂关节当前位置错误
    RobotEventJointEncoderPollustion           = 2110,  // Robot joint encoder pollustion.     机械臂关节编码器污染        建议采取措施:警告性通知
    RobotEventJointEncoderZSignalError         = 2111,  // Robot joint encoder Z signal error. 机械臂关节编码器Z信号错误
    RobotEventJointEncoderCalibrateInvalid     = 2112,  // Robot joint encoder calibrate　invalid. 机械臂关节编码器校准失效
    RobotEventJoint_IMU_SensorInvalid          = 2113,  // Robot joint IMU sensor invalid. 机械臂关节IMU传感器失效
    RobotEventJointTemperatureSensorError      = 2114,  // Robot joint temperature sensor error. 机械臂关节温度传感器出错
    RobotEventJointCanBusError                 = 2115,  // Robot joint CAN BUS error. 机械臂关节CAN总线出错
    RobotEventJointCurrentError                = 2116,  // Robot joint current error. 机械臂关节当前电流错误
    RobotEventJointCurrentPositionError        = 2117,  // Robot joint current position error. 机械臂关节当前位置错误
    RobotEventJointOverSpeed                   = 2118,  // Robot joint over speed. 机械臂关节超速
    RobotEventJointOverAccelerate              = 2119,  // Robot joint over accelerate. 机械臂关节加速度过大错误
    RobotEventJointTraceAccuracy               = 2120,  // Robot joint trace accuracy. 机械臂关节跟踪精度错误
    RobotEventJointTargetPositionOutOfRange    = 2121,  // Robot joint target position out of range.  机械臂关节目标位置超范围
    RobotEventJointTargetSpeedOutOfRange       = 2122,  // Robot joint target speed out of range. 机械臂关节目标速度超范围
    RobotEventJointCollision                   = 2123,  // Robot joint collision. 机械臂碰撞    　　　建议采取措施:暂停当前运动

    RobotEventDataAbnormal                     = 2200,  // Robot data abnormal 机械臂信息异常
    RobotEventRobotTypeError                   = 2201,  // Robot type error 机械臂类型错误
    RobotEventAccelerationSensorError          = 2202,  // Robot acceleration sensor error 机械臂加速度计芯片错误
    RobotEventEncoderLineError                 = 2203,  // Robot encoder line error  机械臂编码器线数错误
    RobotEventEnterDragAndTeachModeError       = 2204,  // Robot enter drag and teach mode error 机械臂进入拖动示教模式错误
    RobotEventExitDragAndTeachModeError        = 2205,  // Robot exit drag and teach mode error 机械臂退出拖动示教模式错误
    RobotEventMACDataInterruptionError         = 2206,  // Robot MAC data interruption error 机械臂MAC数据中断错误

    RobotEventInitAbnormal                     = 2300,  // Robot init abnormal  机械臂初始化异常
    RobotEventDriverEnableFailed               = 2301,  // Robot driver enable failed  机械臂驱动器使能失败
    RobotEventDriverEnableAutoBackFailed       = 2302,  // Robot driver enable auto back failed  机械臂驱动器使能自动回应失败
    RobotEventDriverEnableCurrentLoopFailed    = 2303,  // Robot driver enable current loop failed  机械臂驱动器使能电流环失败
    RobotEventDriverSetTargetCurrentFailed     = 2304,  // Robot driver set target current failed  机械臂驱动器设置目标电流失败
    RobotEventDriverReleaseBrakeFailed         = 2305,  // Robot driver release brake failed  机械臂释放刹车失败
    RobotEventDriverEnablePostionLoopFailed    = 2306,  // Robot driver enable postion loop failed  机械臂使能位置环失败
    RobotEventSetMaxAccelerateFailed           = 2307,  // Robot set max accelerate failed  设置最大加速度失败

    RobotEventSafetyError                      = 2400,  // Robot Safety error  机械臂安全出错
    RobotEventExternEmergencyStop              = 2401,  // Robot extern emergency stop  机械臂外部紧急停止
    RobotEventSystemEmergencyStop              = 2402,  // Robot system emergency stop  机械臂系统紧急停止
    RobotEventTeachpendantEmergencyStop        = 2403,  // Robot teachpendant emergency stop  机械臂示教器紧急停止
    RobotEventControlCabinetEmergencyStop      = 2404,  // Robot control cabinet emergency stop  机械臂控制柜紧急停止
    RobotEventProtectionStopTimeout            = 2405,  // Robot protection stop timeout  机械臂保护停止超时
    RobotEventEeducedModeTimeout               = 2406,  // Robot reduced mode timeout  机械臂缩减模式超时

    RobotEventSystemAbnormal                   = 2500,  // Robot mcu communication error  机械臂mcu通信异常
    RobotEvent_MCU_CommunicationAbnormal       = 2501,  // Robot RS485 communication error  机械臂485通信异常
    RobotEvent485CommunicationAbnormal         = 2502,  // Robot systen abnormal  机械臂系统异常

    RobotEventHardwareErrorNotifyMaximumIndex  = 2599,  // 索引

    //unknown event
    robot_event_unknown
}RobotEventType;


//事件类型
typedef struct{
    RobotEventType  eventType;
    int             eventCode;       //逐步移除
    std::string     eventContent;
}RobotEventInfo;


typedef enum
{
    RobotController_MotionCfgErr,
    RobotController_TrajErr,    
    RobotController_MonitorErr,
    RobotController_InversekinematicsFailure, // to be removed
//    RobotController_OverspeedProtect,
//    RobotController_IkFailure, // to be removed
//    RobotController_OnlineTrajErr,
//    RobotController_OfflineTrajErr,
//    RobotController_InverseDynamicsProtect, // to be removed
    RobotController_StatusException,
}RobotControllerEvent;

typedef enum
{
    subEventNone,
//    MotionCfgErr_MODEJ,
//    MotionCfgErr_MODEL,
//    MotionCfgErr_MODEP,
//    MotionCfgErr_InvalidMODE,
//    MotionCfgErr_NonTrackingBeforeArrival,
//    MotionCfgErr_JointOutOfRange,
//    MotionCfgErr_InvalidReadyWaypointOfMODEP,
//    IkFailure_RelativeMotion,
//    OnlineTrajErr_WaypointGeneration,
//    OnlineTrajErr_typeII,
//    OnlineTrajErr_IkFailure,
//    OnlineTrajErr_InverseDynamicsProtect,
//    OnlineTrajErr_ConveyorOutOfRange,
//    OverspeedProtect_TrajectoryPlanning,
    TrajErr_OnlineTrajPlanning, //conveyor included
    TrajErr_OnlineTypeII,
    TrajErr_InversekinematicsFailure, //conveyor included
    TrajErr_InverseDynamicsProtect, //disabled, for simulated modej only by now
    TrajErr_ConveyorLost,
    TrajErr_ConveyorOutOfRange,
    TrajErr_JointOutOfRange,
    TrajErr_JointOverspeed,
    TrajErr_OfflineTrajPlanning
}RobotControllerSubEventTraj;

typedef enum
{
    MonitorErr_TrackingLost,
    MonitorErr_NoArrivalInTime,
    MonitorErr_CurrentOverload
}RobotControllerSubEventMonitor;

typedef struct
{
    char subEvent;
    char illustration[256];
}motionCfgErr_S;

//for standard DH/zero-joint-pos calibration.
typedef struct
{
    float dA[6];
    float dD[6];    
    float dAlpha[6];
    float dTheta[6];
    float dGearRatio[6];
//    float dZeroOff[6];
}RobotSystemParamCalib_S;

typedef struct
{
   double A3;
   double A4;
   double D1;
   double D2;
   double D5;
   double D6;
}RobotDhPara_S;

typedef enum
{
    //bidirectional
    msgTypeIFBoardRobotType,
    msgTypeIFBoardRobotCalib,
    msgTypeIFBoardRobotIdentify,
    //sent to IfBoard only.
    msgTypeIFBoardToolIdentify,
    msgTypeIFBoardJointFriction,
    msgTypeIFBoardRobotHandGuiding,

    msgTypeCount
}msgTypeIFBoard;
//max 128 bytes
typedef struct
{
    msgTypeIFBoard msgType; //0~4<=>robot type/calib/robot dynamics/tool inertia/tool inertia and frictions.
    short msgLength; //8/64/112/24/48 bytes including 'msgType' and 'msgLength'.
    union
    {
        //to base plane.
        int robotTypeIX;
        short robotCalibPara[30];       //29 used
        //to base plane and split to each joint
        short robotIdentifyPara[54]; //dynIdentifyParaLink(Q10). 38 + 4(rotor inertia) + 6(motor torque constant) + 6(external wrench)
        short toolIdentifyPara[10]; //dynIdentifyParaTool(Q10), sent to IFBoard only.
        ushort jointFrictionPara[30];//dynIdentifyParaTool(Q10), sent to IFBoard only.
        ushort robotHandGuidingPara[48];    //
    }u;
}msgBodyIfBoard_S;

enum dyn_identify_traj
{
    dyn_identify_traj_none = 0,
    dyn_identify_traj_robot, //submode: 0/1 <-> internal/hybrid
    dyn_identify_traj_tool,  //submode: 0/1 <-> tool only/tool+friction
    dyn_identify_traj_tool_abort
};

enum update_board_firmware_cmd
{
    update_master_board_firmware_trans_start = 1,
    update_master_board_firmware_trans_data = 2,
    update_master_board_firmware_trans_end = 3,
    update_slave_board_firmware_trans_start = 4,
    update_slave_board_firmware_trans_data = 5,
    update_slave_board_firmware_trans_end = 6
};

//关节碰撞补偿（范围0.00~0.51度）
typedef struct
{
    double jointOffset[ARM_DOF];
}RobotJointOffset;

//呼叫运动控制函数返回值定义
enum call_robot_motion_func_result
{
    call_robot_motion_func_failed = 0,     //呼叫运动控制函数失败
    call_robot_motion_func_succ_wait_done, //呼叫运动控制函数成功，需要等待完成事件
    call_robot_motion_func_succ_nowait     //呼叫运动控制函数成功，不需要等待完成事件
};



extern double gGearRatio[ARM_DOF];
extern motionCfgErr_S robotMotionCfgErr[MOTION_CONF_ERR_COUNT];
#endif  //AUBO_DEFINITION_H
