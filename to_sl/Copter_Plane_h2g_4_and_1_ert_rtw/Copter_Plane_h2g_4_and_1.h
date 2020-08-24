/*
 * File: Copter_Plane_h2g_4_and_1.h
 *
 * Code generated for Simulink model 'Copter_Plane_h2g_4_and_1'.
 *
 * Model version                  : 1.1054
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Fri Aug 21 14:03:36 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Custom Processor->Custom Processor
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Copter_Plane_h2g_4_and_1_h_
#define RTW_HEADER_Copter_Plane_h2g_4_and_1_h_
#include "rtwtypes.h"
#include <stddef.h>
#include <float.h>
#include <math.h>
#include <string.h>
#ifndef Copter_Plane_h2g_4_and_1_COMMON_INCLUDES_
# define Copter_Plane_h2g_4_and_1_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                           /* Copter_Plane_h2g_4_and_1_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef DEFINED_TYPEDEF_FOR_ENUM_FlightTaskMode_
#define DEFINED_TYPEDEF_FOR_ENUM_FlightTaskMode_

typedef enum {
  NoneFlightTaskMode = 0,              /* Default value */
  GroundStandByMode = 1,
  TakeOffMode = 2,
  Rotor2Fix_Mode = 3,
  HoverAdjustMode = 4,
  HoverUpMode = 5,
  PathFollowMode = 6,
  AirStandByMode = 7,
  GoHomeMode = 8,
  HoverDownMode = 9,
  Fix2Rotor_Mode = 10,
  LandMode = 11,
  FenceRecoverMode = 12,
  SpotCircleMode = 13,
  StallRecovery = 14,
  VerticalMove = 15,
  Test_HeightKeep = 101,
  Test_AttitudeKeep = 102,
  Test_HeadingKeep = 103,
  Test_SpotCircle = 104
} ENUM_FlightTaskMode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ENUM_FlightControlMode_
#define DEFINED_TYPEDEF_FOR_ENUM_FlightControlMode_

typedef enum {
  NoneFlightControlMode = 0,           /* Default value */
  SpotHoverMode,
  HeightKeepMode,
  AttitudeKeepMode,
  HeadingKeepMode,
  PathFollowControlMode,
  CircleHoverMode,
  Move3dMode,
  GroundStandByControlMode,
  DoNothinig,
  OnlyStablizePitchAndRoll,
  RotorGoUpDownBySpeed,
  RotorUnloadToStandby,
  RotorShutDown,
  RotorStable_RollPitchHeight,
  RotorGoUpDownWithHorizonPosFree
} ENUM_FlightControlMode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ENUM_UAVMode_
#define DEFINED_TYPEDEF_FOR_ENUM_UAVMode_

typedef enum {
  NoneUAVMode = 0,                     /* Default value */
  Rotor,
  Fix,
  Rotor2Fix,
  Fix2Rotor
} ENUM_UAVMode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ENUM_ControlMode_
#define DEFINED_TYPEDEF_FOR_ENUM_ControlMode_

typedef enum {
  NoneControlMode = 0,                 /* Default value */
  Auto,
  FCS,
  Manual
} ENUM_ControlMode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ENUM_ComStatus_
#define DEFINED_TYPEDEF_FOR_ENUM_ComStatus_

typedef enum {
  ComNormal = 0,                       /* Default value */
  RemoteOrStationLost
} ENUM_ComStatus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ENUM_Where_
#define DEFINED_TYPEDEF_FOR_ENUM_Where_

typedef enum {
  OnGround = 0,                        /* Default value */
  InTheAir,
  OnWater,
  UnderWater
} ENUM_Where;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ENUM_AutoModeType_
#define DEFINED_TYPEDEF_FOR_ENUM_AutoModeType_

typedef enum {
  NotAutoMode = 0,                     /* Default value */
  AutoProfile,
  SuspendTaskToLand,
  SuspendTaskToHome,
  ManualToAuto,
  SuspendTaskToHomeThenLand,
  Fix2Rotor2HoverAdjust,
  PauseInAir,
  Fix2Rotor2Home2Land
} ENUM_AutoModeType;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BUS_TASK_PATH_OutParam_
#define DEFINED_TYPEDEF_FOR_BUS_TASK_PATH_OutParam_

typedef struct {
  /* 当前目标航点号(home为1号) */
  real_T currentPointNum;

  /* 前置航点号 */
  real_T prePointNum;

  /* 有效航点数量（包括home点） */
  real_T validPathNum;

  /* 目标航向角[rad] */
  real_T headingCmd;

  /* 距离当前目标点的距离 */
  real_T distToGo;

  /* 航线跟踪模式中计算的侧偏距[m] */
  real_T dz;

  /* 目标地速[m/s] */
  real_T groundspeedCmd;

  /* 目标滚转角[rad] */
  real_T rollCmd;

  /* 目标盘旋半径[m] */
  real_T turnRadiusCmd;

  /* 目标高度[m] */
  real_T heightCmd;

  /* 固定翼模式的盘旋中心的纬度、经度，
     或旋翼模式的悬停点 */
  real_T turnCenterLL[2];

  /* 固定翼盘旋时的盘旋半径偏差[m] */
  real_T dR_turn;

  /* 无人机模式，旋翼、固定翼、旋翼转固定翼、固定翼转旋翼 */
  ENUM_UAVMode uavMode;

  /* 飞行任务模式，地面模式、起飞模式等 */
  ENUM_FlightTaskMode flightTaskMode;

  /* 飞行控制模式 */
  ENUM_FlightControlMode flightControlMode;

  /* 飞行流程模式 */
  ENUM_ControlMode AutoManualMode;

  /* 通信状态 */
  ENUM_ComStatus comStatus;

  /* 最大爬升下降速度[m/s] */
  real_T maxClimbSpeed;

  /* 前置航点的纬度、经度、高度 */
  real_T prePathPoint_LLA[3];

  /* 当前航点的纬度、经度、高度 */
  real_T curPathPoint_LLA[3];

  /* 无人机位置，地面、空中 */
  ENUM_Where whereIsUAV;

  /* 自动或手动模式状态 */
  ENUM_AutoModeType typeAutoMode;

  /* 空速指令,[m/s] */
  real_T airspeedCmd;

  /* 执行倾转模式的高度 */
  real_T switchHeight;

  /* 任务中断点的纬度、经度、高度 */
  real_T LLATaskInterrupt[3];

  /* 任务完成标志，当抵达最后一个航点后，标志置true */
  boolean_T isTaskComplete;

  /* 起飞架次 */
  real_T numTakeOff;

  /* 航向是否在地面模式下自旋 */
  boolean_T isHeadingRotate_OnGround;

  /* 是否允许进入暂停模式 */
  boolean_T isAllowedToPause;

  /* 是否需要进行断点记录 */
  boolean_T isNeedToRecordBreak;

  /* 最后一个目标航点号（home点为1，航线航点从2开始） */
  real_T lastTargetPathPoint;
} BUS_TASK_PATH_OutParam;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Bus_wp_
#define DEFINED_TYPEDEF_FOR_Bus_wp_

typedef struct {
  real_T num[11];
  real_T lat[11];
  real_T lon[11];
} Bus_wp;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ENUM_plane_mode_
#define DEFINED_TYPEDEF_FOR_ENUM_plane_mode_

typedef enum {
  V1000 = 0,                           /* Default value */
  V10,
  V10s,
  V10ss
} ENUM_plane_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_sU8UtumGmRbe8OcfmlVmIG_
#define DEFINED_TYPEDEF_FOR_struct_sU8UtumGmRbe8OcfmlVmIG_

typedef struct {
  real_T headingCmd;
  real_T groundspeedCmd;
  real_T heightCmd;
  ENUM_FlightTaskMode flightTaskMode;
  ENUM_FlightControlMode flightControlMode;
  real_T maxClimbSpeed;
  real_T turnCenterLL[2];
  real_T prePathPoint_LLA[3];
  real_T curPathPoint_LLA[3];
  real_T rollCmd;
} struct_sU8UtumGmRbe8OcfmlVmIG;

#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  Bus_wp loc;                          /* '<Root>/loc1' */
  BUS_TASK_PATH_OutParam PathModeOut_sl;/* '<Root>/PathModeOut1' */
  real_T accel_desired[3];             /* '<Root>/accel_desired1' */
  real_T accel_error[3];               /* '<Root>/accel_error1' */
  real_T accel_target[3];              /* '<Root>/accel_target1' */
  real_T accel_xy_input[2];            /* '<Root>/accel_xy_input1' */
  real_T attitude_target_euler_rate[3];
                                      /* '<Root>/attitude_target_euler_rate1' */
  real_T attitude_target_quat[4];      /* '<Root>/attitude_target_quat1' */
  real_T center_WP[2];                 /* '<Root>/center_WP1' */
  real_T curr_pos[2];                  /* '<Root>/curr_pos1' */
  real_T curr_vel[3];                  /* '<Root>/curr_vel1' */
  real_T current_loc[2];               /* '<Root>/current_loc1' */
  real_T groundspeed_vector[2];        /* '<Root>/groundspeed_vector1' */
  real_T loc_origin[2];                /* '<Root>/loc_origin1' */
  real_T pid_vel_xy_derivative[2];     /* '<Root>/pid_vel_xy_derivative1' */
  real_T pid_vel_xy_input[2];          /* '<Root>/pid_vel_xy_input1' */
  real_T pid_vel_xy_integrator[2];     /* '<Root>/pid_vel_xy_integrator1' */
  real_T pitch_factor[4];              /* '<Root>/pitch_factor1' */
  real_T pos_error[3];                 /* '<Root>/pos_error1' */
  real_T pos_target[3];                /* '<Root>/pos_target1' */
  real_T pwm_out[4];                   /* '<Root>/pwm_out1' */
  real_T rate_target_ang_vel[3];       /* '<Root>/rate_target_ang_vel1' */
  real_T roll_factor[4];               /* '<Root>/roll_factor1' */
  real_T rot_body_to_ned[9];           /* '<Root>/rot_body_to_ned1' */
  real_T thrust_rpyt_out[4];           /* '<Root>/thrust_rpyt_out1' */
  real_T vdot_filter[5];               /* '<Root>/vdot_filter1' */
  real_T vel_desired[3];               /* '<Root>/vel_desired1' */
  real_T vel_error[3];                 /* '<Root>/vel_error1' */
  real_T vel_target[3];                /* '<Root>/vel_target1' */
  real_T yaw_factor[4];                /* '<Root>/yaw_factor1' */
  real_T throttle_rpy_mix_desired;     /* '<Root>/throttle_rpy_mix_desired1' */
  real_T AC_ATTITUDE_ACCEL_RP_CONTROLLER;
                       /* '<Root>/AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS1' */
  real_T AC_ATTITUDE_ACCEL_RP_CONTROLL_j;
                       /* '<Root>/AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS1' */
  real_T AC_ATTITUDE_ACCEL_Y_CONTROLLER_;
                        /* '<Root>/AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS1' */
  real_T AC_ATTITUDE_ACCEL_Y_CONTROLLE_g;
                        /* '<Root>/AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS1' */
  real_T AC_ATTITUDE_CONTROL_ANGLE_LIMIT;
                    /* '<Root>/AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX1' */
  real_T AC_ATTITUDE_THRUST_ERROR_ANGLE;
                                  /* '<Root>/AC_ATTITUDE_THRUST_ERROR_ANGLE1' */
  real_T AP_MOTORS_MATRIX_YAW_FACTOR_CCW;
                                 /* '<Root>/AP_MOTORS_MATRIX_YAW_FACTOR_CCW1' */
  real_T AP_MOTORS_MATRIX_YAW_FACTOR_CW;
                                  /* '<Root>/AP_MOTORS_MATRIX_YAW_FACTOR_CW1' */
  real_T ATC_RAT_PIT_D;                /* '<Root>/ATC_RAT_PIT_D1' */
  real_T ATC_RAT_PIT_FF;               /* '<Root>/ATC_RAT_PIT_FF1' */
  real_T ATC_RAT_PIT_FILT;             /* '<Root>/ATC_RAT_PIT_FILT1' */
  real_T ATC_RAT_PIT_I;                /* '<Root>/ATC_RAT_PIT_I1' */
  real_T ATC_RAT_PIT_IMAX;             /* '<Root>/ATC_RAT_PIT_IMAX1' */
  real_T ATC_RAT_PIT_I_inint;          /* '<Root>/ATC_RAT_PIT_I_inint1' */
  real_T ATC_RAT_PIT_P;                /* '<Root>/ATC_RAT_PIT_P1' */
  real_T ATC_RAT_RLL_D;                /* '<Root>/ATC_RAT_RLL_D1' */
  real_T ATC_RAT_RLL_FF;               /* '<Root>/ATC_RAT_RLL_FF1' */
  real_T ATC_RAT_RLL_FILT;             /* '<Root>/ATC_RAT_RLL_FILT1' */
  real_T ATC_RAT_RLL_I;                /* '<Root>/ATC_RAT_RLL_I1' */
  real_T ATC_RAT_RLL_IMAX;             /* '<Root>/ATC_RAT_RLL_IMAX1' */
  real_T ATC_RAT_RLL_I_inint;          /* '<Root>/ATC_RAT_RLL_I_inint1' */
  real_T ATC_RAT_RLL_P;                /* '<Root>/ATC_RAT_RLL_P1' */
  real_T ATC_RAT_YAW_D;                /* '<Root>/ATC_RAT_YAW_D1' */
  real_T ATC_RAT_YAW_FF;               /* '<Root>/ATC_RAT_YAW_FF1' */
  real_T ATC_RAT_YAW_FILT;             /* '<Root>/ATC_RAT_YAW_FILT1' */
  real_T ATC_RAT_YAW_I;                /* '<Root>/ATC_RAT_YAW_I1' */
  real_T ATC_RAT_YAW_IMAX;             /* '<Root>/ATC_RAT_YAW_IMAX1' */
  real_T ATC_RAT_YAW_I_inint;          /* '<Root>/ATC_RAT_YAW_I_inint1' */
  real_T ATC_RAT_YAW_P;                /* '<Root>/ATC_RAT_YAW_P1' */
  real_T EAS2TAS;                      /* '<Root>/EAS2TAS1' */
  real_T EAS_dem;                      /* '<Root>/EAS_dem1' */
  real_T EAS_dem_cm;                   /* '<Root>/EAS_dem_cm1' */
  real_T GRAVITY_MSS;                  /* '<Root>/GRAVITY_MSS1' */
  real_T HD;                           /* '<Root>/HD1' */
  real_T K_A_yaw;                      /* '<Root>/K_A_yaw1' */
  real_T K_D_last_yaw;                 /* '<Root>/K_D_last_yaw1' */
  real_T K_D_yaw;                      /* '<Root>/K_D_yaw1' */
  real_T K_FF_yaw;                     /* '<Root>/K_FF_yaw1' */
  real_T K_FF_yaw_inint;               /* '<Root>/K_FF_yaw_inint1' */
  real_T K_I_yaw;                      /* '<Root>/K_I_yaw1' */
  real_T Kx;                           /* '<Root>/Kx1' */
  real_T L1_damping;                   /* '<Root>/L1_damping1' */
  real_T L1_period;                    /* '<Root>/L1_period1' */
  real_T L1_radius;                    /* '<Root>/L1_radius1' */
  real_T L1_xtrack_i;                  /* '<Root>/L1_xtrack_i1' */
  real_T L1_xtrack_i_gain;             /* '<Root>/L1_xtrack_i_gain1' */
  real_T L1_xtrack_i_gain_prev;        /* '<Root>/L1_xtrack_i_gain_prev1' */
  real_T LOCATION_SCALING_FACTOR;      /* '<Root>/LOCATION_SCALING_FACTOR1' */
  real_T POSCONTROL_ACCELERATION_MIN;/* '<Root>/POSCONTROL_ACCELERATION_MIN1' */
  real_T POSCONTROL_ACCEL_FILTER_HZ;  /* '<Root>/POSCONTROL_ACCEL_FILTER_HZ1' */
  real_T POSCONTROL_ACCEL_XY;          /* '<Root>/POSCONTROL_ACCEL_XY1' */
  real_T POSCONTROL_ACCEL_XY_MAX;      /* '<Root>/POSCONTROL_ACCEL_XY_MAX1' */
  real_T POSCONTROL_ACCEL_Z;           /* '<Root>/POSCONTROL_ACCEL_Z1' */
  real_T POSCONTROL_ACC_Z_D;           /* '<Root>/POSCONTROL_ACC_Z_D1' */
  real_T POSCONTROL_ACC_Z_FILT_HZ;     /* '<Root>/POSCONTROL_ACC_Z_FILT_HZ1' */
  real_T POSCONTROL_ACC_Z_FILT_HZ_c2p;
                                    /* '<Root>/POSCONTROL_ACC_Z_FILT_HZ_c2p1' */
  real_T POSCONTROL_ACC_Z_FILT_HZ_inint;
                                  /* '<Root>/POSCONTROL_ACC_Z_FILT_HZ_inint1' */
  real_T POSCONTROL_ACC_Z_I;           /* '<Root>/POSCONTROL_ACC_Z_I1' */
  real_T POSCONTROL_ACC_Z_IMAX;        /* '<Root>/POSCONTROL_ACC_Z_IMAX1' */
  real_T POSCONTROL_ACC_Z_I_inint;     /* '<Root>/POSCONTROL_ACC_Z_I_inint1' */
  real_T POSCONTROL_ACC_Z_P;           /* '<Root>/POSCONTROL_ACC_Z_P1' */
  real_T POSCONTROL_JERK_RATIO;        /* '<Root>/POSCONTROL_JERK_RATIO1' */
  real_T POSCONTROL_LEASH_LENGTH_MIN;/* '<Root>/POSCONTROL_LEASH_LENGTH_MIN1' */
  real_T POSCONTROL_OVERSPEED_GAIN_Z;/* '<Root>/POSCONTROL_OVERSPEED_GAIN_Z1' */
  real_T POSCONTROL_POS_XY_P;          /* '<Root>/POSCONTROL_POS_XY_P1' */
  real_T POSCONTROL_POS_Z_P;           /* '<Root>/POSCONTROL_POS_Z_P1' */
  real_T POSCONTROL_SPEED;             /* '<Root>/POSCONTROL_SPEED1' */
  real_T POSCONTROL_SPEED_DOWN;        /* '<Root>/POSCONTROL_SPEED_DOWN1' */
  real_T POSCONTROL_SPEED_UP;          /* '<Root>/POSCONTROL_SPEED_UP1' */
  real_T POSCONTROL_THROTTLE_CUTOFF_FREQ;
                                 /* '<Root>/POSCONTROL_THROTTLE_CUTOFF_FREQ1' */
  real_T POSCONTROL_VEL_ERROR_CUTOFF_FRE;
                                /* '<Root>/POSCONTROL_VEL_ERROR_CUTOFF_FREQ1' */
  real_T POSCONTROL_VEL_XY_D;          /* '<Root>/POSCONTROL_VEL_XY_D1' */
  real_T POSCONTROL_VEL_XY_FILT_D_HZ;/* '<Root>/POSCONTROL_VEL_XY_FILT_D_HZ1' */
  real_T POSCONTROL_VEL_XY_FILT_HZ;    /* '<Root>/POSCONTROL_VEL_XY_FILT_HZ1' */
  real_T POSCONTROL_VEL_XY_I;          /* '<Root>/POSCONTROL_VEL_XY_I1' */
  real_T POSCONTROL_VEL_XY_IMAX;       /* '<Root>/POSCONTROL_VEL_XY_IMAX1' */
  real_T POSCONTROL_VEL_XY_I_inint;    /* '<Root>/POSCONTROL_VEL_XY_I_inint1' */
  real_T POSCONTROL_VEL_XY_P;          /* '<Root>/POSCONTROL_VEL_XY_P1' */
  real_T POSCONTROL_VEL_Z_P;           /* '<Root>/POSCONTROL_VEL_Z_P1' */
  real_T STEdotErrLast;                /* '<Root>/STEdotErrLast1' */
  real_T TAS_dem_adj;                  /* '<Root>/TAS_dem_adj1' */
  real_T TAS_state;                    /* '<Root>/TAS_state1' */
  real_T Vz;                           /* '<Root>/Vz1' */
  real_T accel_last_z_cms;             /* '<Root>/accel_last_z_cms1' */
  real_T accel_pitch_max;              /* '<Root>/accel_pitch_max1' */
  real_T accel_roll_max;               /* '<Root>/accel_roll_max1' */
  real_T accel_x;                      /* '<Root>/accel_x1' */
  real_T accel_xy_angle_max;           /* '<Root>/accel_xy_angle_max1' */
  real_T accel_y;                      /* '<Root>/accel_y1' */
  real_T accel_yaw_max;                /* '<Root>/accel_yaw_max1' */
  real_T aerodynamic_load_factor;      /* '<Root>/aerodynamic_load_factor1' */
  real_T air_density_ratio;            /* '<Root>/air_density_ratio1' */
  real_T airspeed_max;                 /* '<Root>/airspeed_max1' */
  real_T airspeed_min;                 /* '<Root>/airspeed_min1' */
  real_T althold_lean_angle_max;       /* '<Root>/althold_lean_angle_max1' */
  real_T ang_vel_pitch_max;            /* '<Root>/ang_vel_pitch_max1' */
  real_T ang_vel_roll_max;             /* '<Root>/ang_vel_roll_max1' */
  real_T ang_vel_yaw_max;              /* '<Root>/ang_vel_yaw_max1' */
  real_T angle_boost_enabled;          /* '<Root>/angle_boost_enabled1' */
  real_T angle_limit_tc;               /* '<Root>/angle_limit_tc1' */
  real_T armed;                        /* '<Root>/armed1' */
  real_T arspeed_filt;                 /* '<Root>/arspeed_filt1' */
  real_T arspeed_temp;                 /* '<Root>/arspeed_temp1' */
  real_T aspeed;                       /* '<Root>/aspeed1' */
  real_T aspeed_c2p;                   /* '<Root>/aspeed_c2p1' */
  real_T aspeed_c2ps;                  /* '<Root>/aspeed_c2ps1' */
  real_T aspeed_cp;                    /* '<Root>/aspeed_cp1' */
  real_T climb_rate;                   /* '<Root>/climb_rate1' */
  real_T climb_rate_cms;               /* '<Root>/climb_rate_cms1' */
  real_T curr_alt;                     /* '<Root>/curr_alt1' */
  real_T current_tilt;                 /* '<Root>/current_tilt1' */
  real_T disable_integrator_pitch;     /* '<Root>/disable_integrator_pitch1' */
  real_T disable_integrator_roll;      /* '<Root>/disable_integrator_roll1' */
  real_T disable_integrator_yaw;       /* '<Root>/disable_integrator_yaw1' */
  real_T dist_min;                     /* '<Root>/dist_min1' */
  real_T dt;                           /* '<Root>/dt1' */
  real_T freeze_ff_z;                  /* '<Root>/freeze_ff_z1' */
  real_T gains_D_pitch;                /* '<Root>/gains_D_pitch1' */
  real_T gains_D_roll;                 /* '<Root>/gains_D_roll1' */
  real_T gains_FF_pitch;               /* '<Root>/gains_FF1' */
  real_T gains_FF_roll;                /* '<Root>/gains_FF_roll1' */
  real_T gains_I_pitch;                /* '<Root>/gains_I_pitch1' */
  real_T gains_I_roll;                 /* '<Root>/gains_I_roll1' */
  real_T gains_P_pitch;                /* '<Root>/gains_P1' */
  real_T gains_P_roll;                 /* '<Root>/gains_P_roll1' */
  real_T gains_imax_pitch;             /* '<Root>/gains_imax_pitch1' */
  real_T gains_imax_roll;              /* '<Root>/gains_imax_roll1' */
  real_T gains_rmax_roll;              /* '<Root>/gains_rmax1' */
  real_T gains_rmax_pitch;             /* '<Root>/gains_rmax_pitch1' */
  real_T gains_tau_pitch;              /* '<Root>/gains_tau_pitch1' */
  real_T gains_tau_roll;               /* '<Root>/gains_tau_roll1' */
  real_T gyro_x;                       /* '<Root>/gyro_x1' */
  real_T gyro_y;                       /* '<Root>/gyro_y1' */
  real_T gyro_z;                       /* '<Root>/gyro_z1' */
  real_T height;                       /* '<Root>/height1' */
  real_T hgt_dem;                      /* '<Root>/hgt_dem1' */
  real_T hgt_dem_adj_last;             /* '<Root>/hgt_dem_adj_last1' */
  real_T hgt_dem_cm;                   /* '<Root>/hgt_dem_cm1' */
  real_T hgt_dem_in_old;               /* '<Root>/hgt_dem_in_old1' */
  real_T hgt_dem_prev;                 /* '<Root>/hgt_dem_prev1' */
  real_T highest_airspeed;             /* '<Root>/highest_airspeed1' */
  real_T imax_yaw;                     /* '<Root>/imax_yaw1' */
  real_T inint;                        /* '<Root>/inint1' */
  real_T inint_hgt;                    /* '<Root>/inint_hgt1' */
  real_T input_tc;                     /* '<Root>/input_tc1' */
  real_T integDTAS_state;              /* '<Root>/integDTAS_state1' */
  real_T integGain;                    /* '<Root>/integGain1' */
  real_T integSEB_state;               /* '<Root>/integSEB_state1' */
  real_T integTHR_state;               /* '<Root>/integTHR_state1' */
  real_T integrator_yaw;               /* '<Root>/integrator_yaw1' */
  real_T inverted_flight;              /* '<Root>/inverted_flight1' */
  real_T k_aileron;                    /* '<Root>/k_aileron1' */
  real_T k_elevator;                   /* '<Root>/k_elevator1' */
  real_T k_rudder;                     /* '<Root>/k_rudder1' */
  real_T k_throttle;                   /* '<Root>/k_throttle1' */
  real_T k_throttle_c2p;               /* '<Root>/k_throttle_c2p' */
  real_T kff_rudder_mix;               /* '<Root>/kff_rudder_mix1' */
  real_T kff_throttle_to_pitch;        /* '<Root>/kff_throttle_to_pitch1' */
  real_T last_Nu;                      /* '<Root>/last_Nu1' */
  real_T last_out_pitch;               /* '<Root>/last_out_pitch1' */
  real_T last_out_roll;                /* '<Root>/last_out_roll1' */
  real_T last_out_yaw;                 /* '<Root>/last_out_yaw1' */
  real_T last_pitch_dem;               /* '<Root>/last_pitch_dem1' */
  real_T last_rate_hp_in_yaw;          /* '<Root>/last_rate_hp_in_yaw1' */
  real_T last_rate_hp_out_yaw;         /* '<Root>/last_rate_hp_out_yaw1' */
  real_T last_throttle_dem;            /* '<Root>/last_throttle_dem1' */
  real_T latAccDem;                    /* '<Root>/latAccDem1' */
  real_T leash;                        /* '<Root>/leash1' */
  real_T leash_down_z;                 /* '<Root>/leash_down_z1' */
  real_T leash_up_z;                   /* '<Root>/leash_up_z1' */
  real_T limit_accel_xy;               /* '<Root>/limit_accel_xy1' */
  real_T limit_pos_up;                 /* '<Root>/limit_pos_up1' */
  real_T loiter_bank_limit;            /* '<Root>/loiter_bank_limit1' */
  real_T loiter_direction;             /* '<Root>/loiter_direction1' */
  real_T maxClimbRate;                 /* '<Root>/maxClimbRate1' */
  real_T maxSinkRate;                  /* '<Root>/maxSinkRate1' */
  real_T max_rate_neg;                 /* '<Root>/max_rate_neg1' */
  real_T minSinkRate;                  /* '<Root>/minSinkRate1' */
  real_T mode;                         /* '<Root>/mode1' */
  real_T mode_L1;                      /* '<Root>/mode_L2' */
  real_T motors_limit_roll_pitch;      /* '<Root>/motors_limit_roll_pitch1' */
  real_T motors_limit_throttle_upper;/* '<Root>/motors_limit_throttle_upper1' */
  real_T motors_limit_yaw;             /* '<Root>/motors_limit_yaw1' */
  real_T nav_pitch_cd;                 /* '<Root>/nav_pitch_cd1' */
  real_T nav_roll_cd;                  /* '<Root>/nav_roll_cd1' */
  real_T p_angle_pitch;                /* '<Root>/p_angle_pitch1' */
  real_T p_angle_roll;                 /* '<Root>/p_angle_roll1' */
  real_T p_angle_yaw;                  /* '<Root>/p_angle_yaw1' */
  real_T p_ff_throttle;                /* '<Root>/p_ff_throttle1' */
  real_T p_plane_c2p;                  /* '<Root>/p_plane_c2p1' */
  real_T p_plane_cp;                   /* '<Root>/p_plane_cp1' */
  real_T p_tail_tilt;                  /* '<Root>/p_tail_tilt1' */
  real_T p_tilt_pitch_target;          /* '<Root>/p_tilt_pitch_target1' */
  real_T pid_accel_z_input;            /* '<Root>/pid_accel_z_input1' */
  real_T pid_accel_z_integrator;       /* '<Root>/pid_accel_z_integrator1' */
  real_T pid_accel_z_reset_filter;     /* '<Root>/pid_accel_z_reset_filter1' */
  real_T pid_info_I_pitch;             /* '<Root>/pid_info_I_pitch1' */
  real_T pid_info_I_roll;              /* '<Root>/pid_info_I_roll1' */
  real_T pid_vel_xy_reset_filter;      /* '<Root>/pid_vel_xy_reset_filter1' */
  real_T pitch;                        /* '<Root>/pitch1' */
  real_T pitch_dem;                    /* '<Root>/pitch_dem1' */
  real_T pitch_in;                     /* '<Root>/pitch_in1' */
  real_T pitch_limit_max_cd;           /* '<Root>/pitch_limit_max_cd1' */
  real_T pitch_limit_min_cd;           /* '<Root>/pitch_limit_min_cd1' */
  real_T pitch_max;                    /* '<Root>/pitch_max1' */
  real_T pitch_max_limit;              /* '<Root>/pitch_max_limit1' */
  real_T pitch_min;                    /* '<Root>/pitch_min1' */
  real_T pitch_target;                 /* '<Root>/pitch_target1' */
  real_T pitch_target_c2p;             /* '<Root>/pitch_target_c2p' */
  real_T pitch_target_p2c;             /* '<Root>/pitch_target_p2c' */
  real_T pitch_target_pilot;           /* '<Root>/pitch_target_pilot1' */
  real_T ptchDamp;                     /* '<Root>/ptchDamp1' */
  real_T pwm_max;                      /* '<Root>/pwm_max1' */
  real_T pwm_min;                      /* '<Root>/pwm_min1' */
  real_T pwm_tail;                     /* '<Root>/pwm_tail' */
  real_T radius;                       /* '<Root>/radius1' */
  real_T rate_bf_ff_enabled;           /* '<Root>/rate_bf_ff_enabled1' */
  real_T rate_pitch_pid_derivative;    /* '<Root>/rate_pitch_pid_derivative1' */
  real_T rate_pitch_pid_input;         /* '<Root>/rate_pitch_pid_input1' */
  real_T rate_pitch_pid_integrator;    /* '<Root>/rate_pitch_pid_integrator1' */
  real_T rate_pitch_pid_reset_filter;/* '<Root>/rate_pitch_pid_reset_filter1' */
  real_T rate_roll_pid_derivative;     /* '<Root>/rate_roll_pid_derivative1' */
  real_T rate_roll_pid_input;          /* '<Root>/rate_roll_pid_input1' */
  real_T rate_roll_pid_integrator;     /* '<Root>/rate_roll_pid_integrator1' */
  real_T rate_roll_pid_reset_filter;  /* '<Root>/rate_roll_pid_reset_filter1' */
  real_T rate_yaw_pid_derivative;      /* '<Root>/rate_yaw_pid_derivative1' */
  real_T rate_yaw_pid_input;           /* '<Root>/rate_yaw_pid_input1' */
  real_T rate_yaw_pid_integrator;      /* '<Root>/rate_yaw_pid_integrator1' */
  real_T rate_yaw_pid_reset_filter;    /* '<Root>/rate_yaw_pid_reset_filter1' */
  real_T recalc_leash_xy;              /* '<Root>/recalc_leash_xy1' */
  real_T recalc_leash_z;               /* '<Root>/recalc_leash_z1' */
  real_T reset_accel_to_lean_xy;       /* '<Root>/reset_accel_to_lean_xy1' */
  real_T reset_desired_vel_to_pos;     /* '<Root>/reset_desired_vel_to_pos1' */
  real_T reset_rate_to_accel_z;        /* '<Root>/reset_rate_to_accel_z1' */
  real_T reverse;                      /* '<Root>/reverse1' */
  real_T roll;                         /* '<Root>/roll1' */
  real_T rollComp;                     /* '<Root>/rollComp1' */
  real_T roll_ff_pitch;                /* '<Root>/roll_ff_pitch1' */
  real_T roll_ff_pitch_inint;          /* '<Root>/roll_ff_pitch_inint1' */
  real_T roll_in;                      /* '<Root>/roll_in1' */
  real_T roll_limit_cd;                /* '<Root>/roll_limit_cd1' */
  real_T roll_limit_cd_inint;          /* '<Root>/roll_limit_cd_inint1' */
  real_T roll_target;                  /* '<Root>/roll_target1' */
  real_T roll_target_pilot;            /* '<Root>/roll_target_pilot1' */
  real_T scaling_speed;                /* '<Root>/scaling_speed1' */
  real_T smoothed_airspeed;            /* '<Root>/smoothed_airspeed1' */
  real_T spdCompFiltOmega;             /* '<Root>/spdCompFiltOmega1' */
  real_T spdWeight;                    /* '<Root>/spdWeight1' */
  real_T tail_tilt;                    /* '<Root>/tail_tilt1' */
  real_T tail_tilt_c2p;                /* '<Root>/tail_tilt_c2p1' */
  real_T tail_tilt_p2c;                /* '<Root>/tail_tilt_p2c1' */
  real_T tail_tilt_rate;               /* '<Root>/tail_tilt_rate1' */
  real_T take_off_land;                /* '<Root>/take_off_land' */
  real_T target_yaw_rate;              /* '<Root>/target_yaw_rate1' */
  real_T thrDamp;                      /* '<Root>/thrDamp1' */
  real_T thr_out_min;                  /* '<Root>/thr_out_min' */
  real_T throttle_avg_max;             /* '<Root>/throttle_avg_max1' */
  real_T throttle_cruise;              /* '<Root>/throttle_cruise1' */
  real_T throttle_cutoff_frequency;    /* '<Root>/throttle_cutoff_frequency1' */
  real_T throttle_dem;                 /* '<Root>/throttle_dem1' */
  real_T throttle_filter;              /* '<Root>/throttle_filter1' */
  real_T throttle_ground;              /* '<Root>/throttle_ground' */
  real_T throttle_hover;               /* '<Root>/throttle_hover1' */
  real_T throttle_in;                  /* '<Root>/throttle_in1' */
  real_T throttle_lower;               /* '<Root>/throttle_lower1' */
  real_T throttle_max;                 /* '<Root>/throttle_max1' */
  real_T throttle_min;                 /* '<Root>/throttle_min1' */
  real_T throttle_off_rate;            /* '<Root>/throttle_off_rate' */
  real_T throttle_rpy_mix;             /* '<Root>/throttle_rpy_mix1' */
  real_T throttle_slewrate;            /* '<Root>/throttle_slewrate1' */
  real_T throttle_thrust_max;          /* '<Root>/throttle_thrust_max1' */
  real_T throttle_upper;               /* '<Root>/throttle_upper1' */
  real_T thrust_boost;                 /* '<Root>/thrust_boost1' */
  real_T thrust_boost_ratio;           /* '<Root>/thrust_boost_ratio1' */
  real_T thrust_slew_time;             /* '<Root>/thrust_slew_time1' */
  real_T timeConstant;                 /* '<Root>/timeConstant1' */
  real_T use_sqrt_controller;          /* '<Root>/use_sqrt_controller1' */
  real_T vel_dot;                      /* '<Root>/vel_dot1' */
  real_T vel_error_input;              /* '<Root>/vel_error_input1' */
  real_T vel_forward_gain;             /* '<Root>/vel_forward_gain1' */
  real_T vel_forward_integrator;       /* '<Root>/vel_forward_integrator' */
  real_T vel_forward_last_pct;         /* '<Root>/vel_forward_last_pct1' */
  real_T vel_forward_min_pitch;        /* '<Root>/vel_forward_min_pitch1' */
  real_T vel_forward_tail_tilt_max;    /* '<Root>/vel_forward_tail_tilt_max1' */
  real_T vertAccLim;                   /* '<Root>/vertAccLim1' */
  real_T weathervane_gain;             /* '<Root>/weathervane_gain1' */
  real_T weathervane_last_output;      /* '<Root>/weathervane_last_output1' */
  real_T weathervane_min_roll;         /* '<Root>/weathervane_min_roll1' */
  real_T yaw;                          /* '<Root>/yaw1' */
  real_T yaw_headroom;                 /* '<Root>/yaw_headroom1' */
  real_T yaw_in;                       /* '<Root>/yaw_in1' */
  real_T yaw_max_c2p;                  /* '<Root>/yaw_max_c2p1' */
  real_T yaw_rate_max;                 /* '<Root>/yaw_rate_max1' */
  real_T mode_state; /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T WP_i;       /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T mode_L1_old;/* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T WP_i_o;     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T uavMode;    /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T Rotor2Fix_delay;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T Rotor2Fix_delay_flag;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T uavMode_p;  /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T Rotor2Fix_delay_b;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T Rotor2Fix_delay_flag_h;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T TakeOffMode_delay;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T TakeOffMode_delay_flag;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T mode_state_j;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T WP_i_m;     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T WP_i_a;     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T uavMode_j;  /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T Rotor2Fix_delay_flag_n;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T uavMode_d;  /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T Rotor2Fix_delay_flag_i;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T TakeOffMode_delay_m;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T TakeOffMode_delay_flag_m;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T mode_state_c;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  real_T WP_i_h;     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  ENUM_FlightTaskMode PathMode;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  ENUM_FlightTaskMode PathMode_j;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  ENUM_FlightTaskMode PathMode_g;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  ENUM_FlightTaskMode PathMode_d;
                     /* '<Root>/input_euler_angle_roll_pitch_euler_rate_yaw4' */
  ENUM_plane_mode plane_mode;          /* '<Root>/plane_mode' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: loc
   * Referenced by: '<Root>/loc1'
   */
  Bus_wp loc1_InitialValue;

  /* Expression: rot_body_to_ned
   * Referenced by: '<Root>/rot_body_to_ned1'
   */
  real_T rot_body_to_ned1_InitialValue[9];
} ConstP;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
  DW *dwork;
};

/* External data declarations for dependent source files */
extern const BUS_TASK_PATH_OutParam
  Copter_Plane_h2g_4_and_1_rtZBUS_TASK_PATH_OutParam;/* BUS_TASK_PATH_OutParam ground */

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void Copter_Plane_h2g_4_and_1_initialize(RT_MODEL *const rtM);
extern void Copter_Plane_h2g_4_and_1_step(RT_MODEL *const rtM);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Copter_Plane_h2g_4_and_1'
 * '<S1>'   : 'Copter_Plane_h2g_4_and_1/input_euler_angle_roll_pitch_euler_rate_yaw4'
 */
#endif                              /* RTW_HEADER_Copter_Plane_h2g_4_and_1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
