function auto_mode_4a1()
%auto flight
%mode auto
global aerodynamic_load_factor
global roll_target
global pitch_target
global target_yaw_rate
global climb_rate_cms
global dt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global pos_target
global curr_pos
global loc_origin
global current_loc
global EAS_dem_cm
global hgt_dem_cm

global aspeed
global p_tilt_pitch_target
global tail_tilt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global vel_desired
global yaw
%%%%%%%%%%L1%%%%%%%%%%%%%%%%%%%%%
global center_WP
global radius
global loiter_direction
global prev_WP
global next_WP
global dist_min
global loc
global L1_radius
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global k_aileron 
global k_elevator 
global k_rudder 
global k_throttle
global throttle_dem
global last_throttle_dem
global nav_pitch_cd
global nav_roll_cd
global yaw_in
global p_plane_c2p
global yaw_max_c2p
global attitude_target_quat
global rot_body_to_ned
global POSCONTROL_ACC_Z_FILT_HZ
global POSCONTROL_ACC_Z_FILT_HZ_c2p
global inint_hgt
global height
global curr_alt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mode auto
global PathModeOut_sl
global airspeed_min
global tail_tilt_c2p
global tail_tilt_p2c
global tail_tilt_rate
global aspeed_c2p
global aspeed_c2ps
persistent WP_i
persistent PathMode
persistent uavMode %0:comper 1:plane
persistent Rotor2Fix_delay
persistent Rotor2Fix_delay_flag
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global disable_integrator_pitch
global disable_integrator_roll
global disable_integrator_yaw
global roll_ff_pitch
global K_FF_yaw

global roll_ff_pitch_inint
global K_FF_yaw_inint
global POSCONTROL_ACC_Z_FILT_HZ_inint
%%%%%%%%%%%%%%%%%%%%%% I %%%%%%%%%%%%%%%%%%%%%
global POSCONTROL_ACC_Z_I_inint
global     POSCONTROL_ACC_Z_I
global     pid_accel_z_reset_filter

global POSCONTROL_VEL_XY_I_inint
global     POSCONTROL_VEL_XY_I
global     pid_vel_xy_reset_filter

global ATC_RAT_PIT_I_inint
global     ATC_RAT_PIT_I
global     rate_pitch_pid_reset_filter

global ATC_RAT_RLL_I_inint
global     ATC_RAT_RLL_I
global     rate_roll_pid_reset_filter

global ATC_RAT_YAW_I_inint
global     ATC_RAT_YAW_I
global     rate_yaw_pid_reset_filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global throttle_filter;
global throttle_in;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global latAccDem
global k_throttle_c2p
global pitch_target_p2c
global throttle_cruise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if isempty(uavMode)
        uavMode = 0;
    end  
    
    if isempty(WP_i)
        WP_i = 1;
    end      
    if isempty(PathMode)
        PathMode = ENUM_FlightTaskMode.NoneFlightTaskMode;
    end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
  if isempty(Rotor2Fix_delay)
        Rotor2Fix_delay = 0;  
  end
  if isempty(Rotor2Fix_delay_flag)
        Rotor2Fix_delay_flag = 0;  
  end   
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
       error_pos=8;   
    if(PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.HoverAdjustMode||PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.TakeOffMode||PathModeOut_sl.flightTaskMode==ENUM_FlightTaskMode.LandMode)
        POSCONTROL_ACC_Z_I=POSCONTROL_ACC_Z_I_inint;
        POSCONTROL_VEL_XY_I=POSCONTROL_VEL_XY_I_inint;
        ATC_RAT_PIT_I=ATC_RAT_PIT_I_inint;
        ATC_RAT_RLL_I=ATC_RAT_RLL_I_inint;
        ATC_RAT_YAW_I=ATC_RAT_YAW_I_inint;    
    else
        POSCONTROL_ACC_Z_I=0;
        POSCONTROL_VEL_XY_I=0;
        ATC_RAT_PIT_I=0;
        ATC_RAT_RLL_I=0;
        ATC_RAT_YAW_I=0;
        pid_accel_z_reset_filter=1;
        pid_vel_xy_reset_filter=1;
        rate_pitch_pid_reset_filter=1;
        rate_roll_pid_reset_filter=1;
        rate_yaw_pid_reset_filter=1;
    end
    if( uavMode==1)%% mode= comper disable plane I
        disable_integrator_pitch=0;
        disable_integrator_roll=0;
        disable_integrator_yaw=0;
        roll_ff_pitch=roll_ff_pitch_inint;
        K_FF_yaw=K_FF_yaw_inint;      
    else
        disable_integrator_pitch=1;
        disable_integrator_roll=1;
        disable_integrator_yaw=1;
        roll_ff_pitch=0;
        K_FF_yaw=0;      
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        switch PathModeOut_sl.flightTaskMode
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.TakeOffMode
                 if(PathMode~=ENUM_FlightTaskMode.TakeOffMode)
                    PathMode=ENUM_FlightTaskMode.TakeOffMode;
                    uavMode=0;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    loc_origin=current_loc;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                    k_throttle=0;
                 else
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( current_loc,loc_origin)*100; 
                 end
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);        
                 copter_run();
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.LandMode
                if(PathMode~=ENUM_FlightTaskMode.LandMode)
                    PathMode=ENUM_FlightTaskMode.LandMode;
                    uavMode=0;
                    loc_origin=current_loc;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                    k_throttle=0;
                else
%                     pos_target(1:2)=[0 0]; 
%                     loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( current_loc,loc_origin)*100; 
                end
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);              
                 copter_run();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            case    ENUM_FlightTaskMode.HoverAdjustMode
                  if(PathMode~=ENUM_FlightTaskMode.HoverAdjustMode)
                    PathMode=ENUM_FlightTaskMode.HoverAdjustMode;
                    uavMode=0;
                    loc_origin=current_loc;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    curr_pos(1:2)=[0 0];
                    pos_target(1:2)=[0 0];
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
                    target_yaw_rate=0;
                    k_throttle=0;
                  else
%                     pos_target(1:2)=[0 0];  
%                     loc_origin=PathModeOut_sl.turnCenterLL(1:2);
                    curr_pos(1:2)=get_vector_xy_from_origin_NE( current_loc,loc_origin)*100; 
                  end               
                 climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                 set_alt_target_from_climb_rate_ffg(PathModeOut_sl.heightCmd,climb_rate_cms, dt, 0);
                 vel_desired(1)=PathModeOut_sl.groundspeedCmd*cos(yaw);
                 vel_desired(2)=PathModeOut_sl.groundspeedCmd*sin(yaw);
                 copter_run();                                                       
            	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.HoverUpMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    center_WP=current_loc;                     
                end 
                 if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos         
                     climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                 elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                     climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;                
                 else
                     climb_rate_cms=0;
                     hgt_dem_cm=PathModeOut_sl.heightCmd;
                 end 
%                  center_WP=PathModeOut_sl.turnCenterLL(1:2);
                 update_loiter( center_WP,   radius,   loiter_direction)
                 plane_run_4a1();  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.HoverDownMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    center_WP=current_loc;                     
                end 
                 if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos         
                     climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                 elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                     climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;                
                 else
                     climb_rate_cms=0;
                     hgt_dem_cm=PathModeOut_sl.heightCmd;
                 end
%                center_WP=PathModeOut_sl.turnCenterLL(1:2);
                update_loiter( center_WP,   radius,   loiter_direction)  
                if(PathModeOut_sl.rollCmd==0)
                    latAccDem=0;
                end
                plane_run_4a1();                 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.AirStandByMode
                if(PathMode~=ENUM_FlightTaskMode.HoverUpMode)
                    PathMode=ENUM_FlightTaskMode.HoverUpMode;
                    uavMode=1;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    center_WP=current_loc;                     
                end 
                 if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos         
                     climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                 elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                     climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;                
                 else
                     climb_rate_cms=0;
                     hgt_dem_cm=PathModeOut_sl.heightCmd;
                 end 
%                center_WP=PathModeOut_sl.turnCenterLL(1:2); 
                update_loiter( center_WP,   radius,   loiter_direction) 
                plane_run_4a1();  
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.Rotor2Fix_Mode 
                if(PathMode~=ENUM_FlightTaskMode.Rotor2Fix_Mode)
                    PathMode=ENUM_FlightTaskMode.Rotor2Fix_Mode;
                    uavMode=0;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    inint_hgt=1;
                    hgt_dem_cm=height*100;
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225
                    target_yaw_rate=0; 
                    roll_target=0;
                    Rotor2Fix_delay_flag=0;
                    Rotor2Fix_delay=0;
                end
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 if(uavMode==1)
                     if(Rotor2Fix_delay_flag)
                            Rotor2Fix_delay_flag=1;
                            nav_roll_cd=0;
                            update_50hz();
                            update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor);
                            calc_nav_pitch();                
                            calc_throttle();
                            stabilize();
                            output_to_motors_plane_4a1();
                     else
                            k_throttle=throttle_cruise * 0.01+k_throttle_c2p; 
                            update_speed(aerodynamic_load_factor);
                            pitch_target=0;
                            pitch_target_temp=0;
                            input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                            rate_controller_run();
                            %%%%  
                            nav_pitch_cd=pitch_target_temp;
                            nav_roll_cd=roll_target;
                            stabilize()
                            throttle_in=0;
                            yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                            POSCONTROL_ACC_Z_FILT_HZ=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                            AP_MotorsMulticopter_output_4a1();  
                            if(throttle_filter<0.1)
                                  Rotor2Fix_delay_flag=1; 
                                  throttle_dem=throttle_cruise * 0.01+k_throttle_c2p;
                                  last_throttle_dem=throttle_cruise * 0.01+k_throttle_c2p;
                            end                         
                     end                 
                 else
                      k_throttle=throttle_cruise * 0.01+k_throttle_c2p;                    
                      update_speed(aerodynamic_load_factor);
                     if(aspeed>aspeed_c2ps)
                         %%%%
                         pitch_target=0;
                         pitch_target_temp=0;
                         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                         rate_controller_run();
                         %%%%  
                        nav_pitch_cd=pitch_target_temp;
                        nav_roll_cd=roll_target;
                        stabilize()
                        throttle_in=0;
                        yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        POSCONTROL_ACC_Z_FILT_HZ=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        AP_MotorsMulticopter_output_4a1();  
                        uavMode=1; 
                     elseif(aspeed>aspeed_c2p)
                         %%%%
                         if(pitch_target>0)
                             pitch_target=0;
                         end
                         pitch_target_temp=pitch_target+p_tilt_pitch_target*-3000;
                         update_z_controller();
                         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                         rate_controller_run();
                         %%%%  
                        nav_pitch_cd=pitch_target_temp;
                        nav_roll_cd=roll_target;
                        stabilize()
                        k_aileron=k_aileron*p_plane_c2p;
                        k_elevator=k_elevator*p_plane_c2p;
                        k_rudder=k_rudder*p_plane_c2p;
                        yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                        POSCONTROL_ACC_Z_FILT_HZ=POSCONTROL_ACC_Z_FILT_HZ_c2p;
                        AP_MotorsMulticopter_output_4a1();
                     else
                         if(pitch_target>0)
                             pitch_target=0;
                         end
                         pitch_target_temp=pitch_target+p_tilt_pitch_target*-3000;
                         update_z_controller();
                         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target_temp,   target_yaw_rate);
                         rate_controller_run();
                         %%%%
                         POSCONTROL_ACC_Z_FILT_HZ=POSCONTROL_ACC_Z_FILT_HZ_inint;
                         k_aileron=0;
                         k_elevator=0;
                         k_rudder=0;
                         AP_MotorsMulticopter_output_4a1();
                     end     
                 end         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case    ENUM_FlightTaskMode.Fix2Rotor_Mode
                 if(PathMode~=ENUM_FlightTaskMode.Fix2Rotor_Mode)
                    PathMode=ENUM_FlightTaskMode.Fix2Rotor_Mode;
                    uavMode=0;
                    pos_target(3) = curr_alt;
                    vel_desired(3)=0;
                    attitude_target_quat=from_rotation_matrix(rot_body_to_ned);%20200225
                    target_yaw_rate=0;  
                    tail_tilt=tail_tilt_p2c;
                    roll_target=0;
                    pitch_target=pitch_target_p2c;
                    k_throttle=0;
                    throttle_filter=0;
                    throttle_in=0;
                end
                 update_z_controller();
                 input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
                 rate_controller_run();
                 if(aspeed>aspeed_c2p)
                    nav_pitch_cd=pitch_target;
                    nav_roll_cd=roll_target;
                    stabilize()
                    k_aileron=k_aileron*p_plane_c2p;
                    k_elevator=k_elevator*p_plane_c2p;
                    k_rudder=k_rudder*p_plane_c2p;
                    yaw_in=constrain_value(yaw_in,-yaw_max_c2p,yaw_max_c2p);
                    POSCONTROL_ACC_Z_FILT_HZ=POSCONTROL_ACC_Z_FILT_HZ_inint;
                    AP_MotorsMulticopter_output_4a1();
                 else          
                     POSCONTROL_ACC_Z_FILT_HZ=POSCONTROL_ACC_Z_FILT_HZ_inint;
                     k_aileron=0;
                     k_elevator=0;
                     k_rudder=0;
                     AP_MotorsMulticopter_output_4a1();
                 end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
            case    ENUM_FlightTaskMode.PathFollowMode               
                if(PathMode~=ENUM_FlightTaskMode.PathFollowMode)
                     PathMode=ENUM_FlightTaskMode.PathFollowMode;
                     uavMode=1;
                     inint_hgt=1;
                     center_WP=current_loc;                     
                end 
                 if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos         
                     climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                 elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                     climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;                
                 else
                     climb_rate_cms=0;
                     hgt_dem_cm=PathModeOut_sl.heightCmd;
                 end                    
                     prev_WP=[loc.lat(WP_i) loc.lon(WP_i)];
                     next_WP=[loc.lat(WP_i+1) loc.lon(WP_i+1)];        
                    AB = get_distance_NE(next_WP,current_loc);
                    AB_length = norm(AB,2);
                    if((AB_length<L1_radius))
                        if((loc.num(WP_i+2,1)~=99))       
                            WP_i=WP_i+1;      
                        else               
                            WP_i=2;
                        end
                    end
                    update_waypoint( prev_WP,  next_WP,  dist_min)
                    plane_run_4a1();   
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
            case    ENUM_FlightTaskMode.GoHomeMode 
                 if(PathMode~=ENUM_FlightTaskMode.GoHomeMode)
                     PathMode=ENUM_FlightTaskMode.GoHomeMode;
                     inint_hgt=1;
                     center_WP=current_loc;                     
                end 
                 if (PathModeOut_sl.heightCmd-hgt_dem_cm)>error_pos         
                     climb_rate_cms=PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;
                 elseif (PathModeOut_sl.heightCmd-hgt_dem_cm)<-error_pos
                     climb_rate_cms=-PathModeOut_sl.maxClimbSpeed;
                     hgt_dem_cm=hgt_dem_cm+dt*climb_rate_cms;                
                 else
                     climb_rate_cms=0;
                     hgt_dem_cm=PathModeOut_sl.heightCmd;
                 end                    
                     prev_WP=[loc.lat(WP_i) loc.lon(WP_i)];
                     next_WP=[loc.lat(1) loc.lon(1)];        
                    update_waypoint( prev_WP,  next_WP,  dist_min)
                    plane_run_4a1();                   
            otherwise         
                 copter_run();
        end 
end

