%mode_Multicopter
load bus_comper.mat
load IOBusInfo_V1000_20200807.mat
pianzhuanjiao
m_kg=5;

% global POSCONTROL_POS_Z_P
% global POSCONTROL_ACCEL_Z
% global POSCONTROL_SPEED_DOWN
% global POSCONTROL_SPEED_UP
% global dt

    POSCONTROL_ACC_Z_DT=0.0025;    % vertical acceleration controller dt default
    gains_rmax_roll=0;   
    thrust_slew_time=0.3;%%%%%%%%%%%%%%%%����ʱ��
    throttle_slewrate=100;%%%%%%%%%%%%%%%%TECS ���ű仯��
    yaw_rate_max=50;
    plane_mode=ENUM_plane_mode.V10;
    pwm_tail=1000;
    vel_forward_integrator=0;
    
 p_angle_pitch                         = 3                            ;
 p_angle_roll                          = 3                            ;
 p_angle_yaw                           = 3                            ;
 ATC_RAT_PIT_D                         = 0                            ;
 ATC_RAT_PIT_FF                        = 0                            ;
 ATC_RAT_PIT_FILT                      = 20                           ;
 ATC_RAT_PIT_I_inint                   = 0.1                          ;
 ATC_RAT_PIT_IMAX                      = 0.25                         ;
 ATC_RAT_PIT_P                         = 0.2                          ;
 ATC_RAT_RLL_D                          = 0                           ;
 ATC_RAT_RLL_FF                         = 0                           ;
 ATC_RAT_RLL_FILT                       = 20                          ;
 ATC_RAT_RLL_I_inint                    = 0.1                         ;
 ATC_RAT_RLL_IMAX                       = 0.25                        ;
 ATC_RAT_RLL_P                          = 0.21                        ;
 ATC_RAT_YAW_D                          = 0                           ;
 ATC_RAT_YAW_FF                         = 0                           ;
 ATC_RAT_YAW_FILT                       = 5                           ;
 ATC_RAT_YAW_I_inint                    = 0                           ;
 ATC_RAT_YAW_IMAX                       = 0                           ;
 ATC_RAT_YAW_P                          = 0.19                        ;
 POSCONTROL_POS_Z_P                     = 1                           ;
 POSCONTROL_VEL_Z_P                     = 1                           ;
 POSCONTROL_ACC_Z_P                     = 0.9                         ;
 POSCONTROL_ACC_Z_I_inint               = 0.1                        ;
 POSCONTROL_ACC_Z_D                     = 0.3                         ;
 POSCONTROL_ACC_Z_IMAX                  = 100                         ;
 POSCONTROL_ACC_Z_FILT_HZ_inint         = 4                         ;
 POSCONTROL_THROTTLE_CUTOFF_FREQ        = 3                           ;
 gains_tau_pitch                        = 1                           ;
 gains_P_pitch                          = 0.9                        ;
 gains_D_pitch                          = 0.35                        ;
 gains_I_pitch                          = 0.02                        ;
 roll_ff_pitch_inint                    = 0.85                        ;
 gains_imax_pitch                       = 3000                        ;
 gains_tau_roll                         = 0.5                         ;
 gains_P_roll                           = 1.07                        ;
 gains_D_roll                           = 0.22                        ;
 gains_I_roll                           = 0                           ;
 gains_imax_roll                        = 3000                        ;
 gains_FF_roll                          = 0                           ;
 K_A_yaw                                = 0                           ;
 K_I_yaw                                = 0.05                        ;
 K_D_yaw                                = 0.25                        ;
 K_FF_yaw_inint                         = 1                           ;
 imax_yaw                               = 1500                        ;
 current_tilt                           = 0.35                        ;
 throttle_hover                         = 0.6                         ;
 maxClimbRate                           = 5                          ;
 minSinkRate                            = 3                           ;
 timeConstant                           = 3                            ;
 thrDamp                                = 0.7                         ;
 integGain                              = 0.25                        ;
 vertAccLim                             = 3.5                         ;
 spdCompFiltOmega                       = 2                           ;
 rollComp                               = 12                          ;
 spdWeight                              = 1                           ;
 ptchDamp                               = 0.7                         ;
 maxSinkRate                            = 7                           ;
 throttle_cruise                        = 40                          ;
 arspeed_filt                           = 5                           ;
 EAS_dem_cm                             = 1900                        ;
 POSCONTROL_POS_XY_P                    = 1.2                         ;
 POSCONTROL_VEL_XY_P                    = 1.2                         ;
 POSCONTROL_VEL_XY_I_inint              = 0.1                         ;
 POSCONTROL_VEL_XY_D                    = 0.12                        ;
 POSCONTROL_VEL_XY_IMAX                 = 1000                        ;
 POSCONTROL_VEL_XY_FILT_HZ              = 2                           ;
 POSCONTROL_VEL_XY_FILT_D_HZ            = 5                           ;
 L1_period                              = 15                          ;
 L1_damping                             = 0.75                        ;
 L1_xtrack_i_gain                       = 0                           ;
 p_tail_tilt                            = 3                           ;
 p_tilt_pitch_target                    = 0.2                         ;
 aspeed_c2p                             = 8                           ;
 p_plane_c2p                            = 0.6                         ;
 yaw_max_c2p                            = 0.05                        ;
 weathervane_min_roll                   = 5                           ;
 weathervane_gain                       = 0.2                         ;
 POSCONTROL_ACC_Z_FILT_HZ_c2p           = 0.2                         ;
 kff_rudder_mix                         = 1                           ;
 radius                                 = 60                          ;
 loiter_direction                       = 1                           ;
 tail_tilt_c2p                          = -2800                       ;
 tail_tilt_p2c                          = -1300                       ;
 tail_tilt_rate                         = 1500                        ;
 aspeed_c2ps                            = 15                          ;
 POSCONTROL_ACCEL_FILTER_HZ             = 10                          ;
 POSCONTROL_VEL_ERROR_CUTOFF_FREQ       = 1.5                         ;
 roll_limit_cd_inint                    = 3500                        ;
 cutoff_freq_a                          = 25                          ;
 cutoff_freq_g                          = 35                          ; 
 p_ff_throttle                          = 0.5                           ;
 vel_forward_gain                       = 1                           ;
 vel_forward_min_pitch                  = -30                         ; 
 aspeed_cp                              = 30                          ;
 p_plane_cp                             = 0.4                         ;
 POSCONTROL_SPEED                       = 300                         ;
 POSCONTROL_ACCEL_XY                    = 100                         ;
 POSCONTROL_ACCEL_Z                     = 100                         ;
 thr_out_min                            = 0.4                         ;       
 pitch_target_p2c                       = 500                         ;
 k_throttle_c2p                         = 0.4                         ;
 throttle_off_rate                      = 0.03                        ;
 throttle_ground                        = 0.55                        ;
 yaw_in_max                             = 1.00                        ;
 pitch_target_c2p                       = 000                         ;

    take_off_land=0;
    ff_throttle=0;
    GRAVITY_MSS=9.80665;
    ATC_RAT_PIT_I   = 0.0;
    ATC_RAT_RLL_I    =0.0;
    ATC_RAT_YAW_I    =0.0;
    K_FF_yaw=0; 
    roll_ff_pitch=0;
    POSCONTROL_ACC_Z_I=0.0;    % vertical acceleration controller I gain default
    POSCONTROL_ACC_Z_FILT_HZ=0;    % vertical acceleration controller input filter default
    POSCONTROL_VEL_XY_I=0; % horizontal velocity controller I gain default
    disable_integrator_pitch=0;
    disable_integrator_roll=0;
    disable_integrator_yaw=0;
    tail_tilt=0;
    inint_hgt=1;   
    pitch_max=0;
    pitch_min=0;
    hgt_dem_cm=0;
    arspeed_temp=0;
    loiter_bank_limit=0;
    weathervane_last_output=0;
    
    nav_pitch_cd=0;
    nav_roll_cd=0;
    inverted_flight=0;
    k_rudder=0;
    k_aileron=0;
    k_throttle=0.0;
    k_elevator=0;
    desired_rate_pitch=0;
    desired_rate_roll=0;
    
    kff_throttle_to_pitch=0;
    smoothed_airspeed=0;
    aerodynamic_load_factor=1;
    vdot_filter=zeros(5,1);
    rot_body_to_ned=eye(3,3);
    imu_filt=10;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 vel_forward_last_pct=0;
 vel_forward_tail_tilt_max=2000;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Lux=310/1000;
Luy=210/1000;
Ldx=205/1000;
Ldy=355.5/1000;
Lu=hypot(Lux,Luy);
Ld=hypot(Ldx,Ldy);
Ku=32;
% Kd=18;
Kd=Ku*Luy/Ldy;
Kx=(Ku-Kd)/(Ku+Kd);
Qd=10;
Qu=asind(Kd*sind(Qd)*Ld/(Ku*Lu));
Kc=(Ku*Lux)/(Ldx*Kd);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%sl

PathModeOut_sl.headingCmd=0;
PathModeOut_sl.groundspeedCmd=0;
PathModeOut_sl.heightCmd=0.1;
PathModeOut_sl.flightTaskMode=ENUM_FlightTaskMode.Rotor2Fix_Mode;
PathModeOut_sl.maxClimbSpeed=0;
PathModeOut_sl.turnCenterLL=[0 0];
PathModeOut_sl.prePathPoint_LLA=[0 0 0];
PathModeOut_sl.curPathPoint_LLA=[0 0 0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mode=0;
    mode_state=0;
    ts=0;
    dt=0.012;
    pwm_max=2000;
    pwm_min=1000;
    pwm_out=[1000 1000 1000 1000];
    highest_airspeed=30;
    scaling_speed=17;
    airspeed_max=23;
    airspeed_min=15;

    TASmax=23;
    TASmin=15;
    THRmaxf=1;
    THRminf=0;
    throttle_min=0;
    throttle_max=100; 
    pitch_limit_min_cd=-1500;
    pitch_limit_max_cd=2000;
    pitch_max_limit=90;
    roll_limit_cd=2500;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% wind %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    HD=180/pi;
    pitch=0;
    roll=0;
    yaw=0;
    wy=0;
    wx=0;
    wz=0;
    
     curr_vel=[0 0 0];
     curr_pos=[0 0 ];

    gyro_x=0;
    gyro_y=0;
    gyro_z=0;

    accel_x=0;
    accel_y=0;    
    accel_z=0;
    VN=0;
    VE=0;  
    Vz=0;
    EAS=17;
    aspeed=17;
    EAS2TAS=1;   
    groundspeed_vector=[0.1 0.1];
    current_loc=[40,100]*1e7;
    loc_origin=[40,100]*1e7;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    center_WP=[40,100]*1e7;
    L1_radius=60;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    prev_WP=[40,100]*1e7;
    next_WP=[40,100.01]*1e7;
    dist_min=50;
%     loc=[0,      40*1e7,     100*1e7;
%          1,      40.01*1e7,  100.01*1e7;
%          2,      40.02*1e7,  100.01*1e7;
%          3,      40.02*1e7,  100.02*1e7;
%          4,      40.01*1e7,  100.02*1e7;
%          5,      40.01*1e7,  100.01*1e7;
%          99,     0*1e7,      0*1e7;    ];
    loc.num=[
0.0000000e+00
1.0000000e+00
2.0000000e+00
3.0000000e+00
4.0000000e+00
5.0000000e+00
9.9000000e+01
9.9000000e+01
9.9000000e+01
9.9000000e+01
9.9000000e+01

 ];

    loc.lat=[
2.9112732e+01
2.9113876e+01
2.9113876e+01
2.9114450e+01
2.9114450e+01
2.9113876e+01
4.0010000e+01
0.0000000e+00
0.0000000e+00
0.0000000e+00
0.0000000e+00
]'*1e7;
    loc.lon=[
1.0490368e+02
1.0490256e+02
1.0490623e+02
1.0490623e+02
1.0490256e+02
1.0490256e+02
1.0001000e+02
0.0000000e+00
0.0000000e+00
0.0000000e+00
0.0000000e+00
]'*1e7;
    
    mode_L1=0;
    roll_target_pilot=0;
    pitch_target_pilot=0; 
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AC_AttitudeControl matlab sim  
slew_yaw=6000;
accel_yaw_max=18000;
rate_bf_ff_enabled=1;
accel_roll_max=72000;
accel_pitch_max=30000; 
angle_boost_enabled=1;

angle_limit_tc=1;
ang_vel_roll_max=0;
ang_vel_pitch_max=0;
ang_vel_yaw_max=0;
input_tc=0.3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS= radians(40.0) ;    %minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS= radians(720.0);   %maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS= radians(10.0) ;   % minimum body-frame acceleration limit for the stability controller (for yaw axis)
AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS=radians(120.0) ;   % maximum body-frame acceleration limit for the stability controller (for yaw axis)
AC_ATTITUDE_THRUST_ERROR_ANGLE=radians(30.0);               %Thrust angle error above which yaw corrections are limited
AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX=0.8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ATC_ACCEL_P_MAX = 36397.976562;
ATC_ACCEL_R_MAX = 33111.574219;
ATC_ACCEL_Y_MAX = 36397.976562;

rate_yaw_pid_reset_filter=0;
rate_yaw_pid_input=0;
rate_yaw_pid_derivative=0;
rate_yaw_pid_integrator=0;
motors_limit_yaw=0;

rate_roll_pid_reset_filter=0;
rate_roll_pid_input=0;
rate_roll_pid_derivative=0;
rate_roll_pid_integrator=0;
motors_limit_roll_pitch=0;

rate_pitch_pid_reset_filter=0;
rate_pitch_pid_input=0;
rate_pitch_pid_derivative=0;
rate_pitch_pid_integrator=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    POSCONTROL_ACCELERATION_MIN=50.0;% minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
%     POSCONTROL_ACCEL_XY=        100.0;%default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
    POSCONTROL_ACCEL_XY_MAX=    458;%max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
    POSCONTROL_STOPPING_DIST_UP_MAX=         300.0;%max stopping distance (in cm) vertically while climbing
    POSCONTROL_STOPPING_DIST_DOWN_MAX=       200.0;%max stopping distance (in cm) vertically while descending

%     POSCONTROL_SPEED=           300.0;%default horizontal speed in cm/s
    POSCONTROL_SPEED_DOWN=     -300.0;%default descent rate in cm/s
    POSCONTROL_SPEED_UP=        300.0;%default climb rate in cm/s

%     POSCONTROL_ACCEL_Z=         250.0;%default vertical acceleration in cm/s/s.

    POSCONTROL_LEASH_LENGTH_MIN=100.0;%minimum leash lengths in cm

    POSCONTROL_DT_50HZ=         0.02;% time difference in seconds for 50hz update rate
    POSCONTROL_DT_400HZ=        0.0025;% time difference in seconds for 400hz update rate

    POSCONTROL_ACTIVE_TIMEOUT_US=200000 ;% position controller is considered active if it has been called within the past 0.2 seconds

    % low-pass filter on velocity error (unit: hz)
    % low-pass filter on acceleration (unit: hz)
    POSCONTROL_JERK_RATIO=      1.0;% Defines the time it takes to reach the requested acceleration

    POSCONTROL_OVERSPEED_GAIN_Z=2.0;% gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range
    
    accel_xy_filt_hz=POSCONTROL_ACCEL_FILTER_HZ;

 recalc_leash_z=1;
 limit_pos_down=0;
 limit_pos_up=0;
 pos_error=[0 0 0];
 pos_target=[0 0 0];
 vel_target=[0 0 0];
 limit_vel_up=0;
 limit_vel_down=0;
 use_desvel_ff_z=1;
 reset_rate_to_accel_z=1;
 curr_alt=0;
 vel_desired=[0 0 0];
 vel_last=[0 0 0];
 accel_desired=[0 0 0];
 freeze_ff_z=0;
 vel_error=[0 0 0];
 vel_error_input=0;
 accel_target=[0 0 0];
 reset_accel_to_throttle=1;
 accel_error=[0 0 0];
 z_accel_meas=0;
 pid_accel_z_reset_filter=1;
 pid_accel_z_input=0;
 pid_accel_z_derivative=0;
 pid_accel_z_integrator=0;
 throttle_lower=0;
 throttle_upper=0;
 throttle_input=0;
 %%%%%%%%%%%%%%%%%%
 roll_target=0;
 pitch_target=0;
 yaw_target=0;
 target_yaw_rate=0;
 
 attitude_target_euler_angle=[roll_target pitch_target yaw_target]*0.01/HD;
 attitude_target_quat=from_euler(attitude_target_euler_angle);

attitude_target_ang_vel=[0 0 0];
attitude_target_euler_rate=[0 0 0];
attitude_ang_error=[1 0 0 0];
use_sqrt_controller=1;
 
 %%%%%%%%%%%%%%%%%%%%%%
 throttle_rpy_mix=0;

 pid_vel_xy_reset_filter=1;
 pid_vel_xy_input=[0 0];
 pid_vel_xy_derivative=[0 0];
 pid_vel_xy_integrator=[0 0];
 limit_accel_xy=0;
 motors_limit_throttle_upper=0;
 reset_accel_to_lean_xy=1;
 accel_xy_input=[0 0];
 accel_xy_angle_max=2000;%%dai cha

 recalc_leash_xy=1;
 reset_desired_vel_to_pos=1;
 accel_last_z_cms=0;
 is_active_z=1;
 is_active_xy=1;

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 inint=1;
 AP_MOTORS_MATRIX_YAW_FACTOR_CW=-1;
 AP_MOTORS_MATRIX_YAW_FACTOR_CCW=1;

roll_factor=[0 0 0 0];
pitch_factor=[0 0 0 0];
yaw_factor=[0 0 0 0];

limit_roll_pitch=0;
limit_yaw=0;
yaw_headroom=0;
air_density_ratio=1;
thrust_boost=0;

angle_boost=0;
throttle_avg_max=1;
throttle_cutoff_frequency=1;
throttle_thrust_max=1;
althold_lean_angle_max=0;
throttle_rpy_mix_desired=0;
AC_ATTITUDE_CONTROL_MAX=5;
roll_in=0;
pitch_in=0;
yaw_in=0;
armed=1;
% throttle_hover=0.4816;
% throttle_in=0.4816;
% throttle_filter=throttle_in;
% pwm_out=ones(1,4)*(throttle_hover*1000+1000);

throttle_in=0.4816;
throttle_filter=0;
thrust_boost_ratio=0;
thrust_rpyt_out=[0 0 0 0];
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
thrust_error_angle=0;
rate_target_ang_vel=[0 0 0];
attitude_error_vector=[0 0 0];
climb_rate_cms=0;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AP_PitchController

    gains_rmax_pitch=0;
    max_rate_neg=0;

    last_out_pitch=0;
    pid_info_I_pitch=0;
    pid_info_P_pitch=0;
    pid_info_FF_pitch=0;
    pid_info_D_pitch=0;
    pid_info_desired_pitch=0;
    pid_info_actual_pitch=0;
    gains_FF_pitch=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_RollController

    last_out_roll=0;
    pid_info_I_roll=0;
    pid_info_P_roll=0;
    pid_info_FF_roll=0;
    pid_info_D_roll=0;
    pid_info_desired_roll=0;
    pid_info_actual_roll=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%  AP_YawController 
	% @Param: SLIP
	% @DisplayName: Sideslip control gain
	% @Description: Gain from lateral acceleration to demanded yaw rate for aircraft with enough fuselage area to detect lateral acceleration and sideslips. Do not enable for flying wings and gliders. Actively coordinates flight more than just yaw damping. Set after YAW2SRV_DAMP and YAW2SRV_INT are tuned.
	% @Range: 0 4
	% @Increment: 0.25
    % @User: Advanced
% 	AP_GROUPINFO("SLIP",    0, AP_YawController, _K_A,    0),

    
    K_D_last_yaw=0;
    integrator_yaw=0;
    last_out_yaw=0;
    pid_info_I_yaw=0;
    pid_info_D_yaw=0;
    last_rate_hp_out_yaw=0;
    last_rate_hp_in_yaw=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_TECS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SKE_dem=0;
    SPE_dem=0;
    SPE_est=0;
    SKE_est=0;
    STE_error=0;
    SPEdot_dem=0;
    SKEdot_dem=0;
    STEdot_min=0;
    STEdot_max=0;
    SPEdot=0;
    SKEdot=0;
    STEdotErrLast=0;
    throttle_dem=0;
    last_throttle_dem=0;
    integTHR_state=0;
    hgt_dem=0;
    PITCHmaxf=0;
    PITCHminf=0;
    TAS_rate_dem=0;
    hgt_dem_in_old=0;
    max_sink_rate=0;
    hgt_dem_prev=0;
    hgt_dem_adj=0;
    hgt_dem_adj_last=0;
    hgt_rate_dem=0;
    height=0;
    climb_rate=0;
    pitch_dem=0;
    integSEB_state=0;
    pitch_dem_unc=0;
    last_pitch_dem=0;

    EAS_dem=17;
    TAS_dem=17;
    TAS_dem_adj=TAS_dem;
    TAS_state=17;
    integDTAS_state=0;
    vel_dot=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%AP_L1_Control

 LOCATION_SCALING_FACTOR = 0.011131884502145034;
 LOCATION_SCALING_FACTOR_INV = 89.83204953368922;
    target_bearing_cd=0;
    L1_dist=0;
    crosstrack_error=0;
    nav_bearing=0;
    L1_xtrack_i_gain_prev=0;
    L1_xtrack_i=0;
    last_Nu=0;
    latAccDem=0;
    WPcircle=0;
    bearing_error=0;
    data_is_stale=0;
    reverse=0;



