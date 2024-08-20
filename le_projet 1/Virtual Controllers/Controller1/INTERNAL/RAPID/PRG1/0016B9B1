MODULE Module1

    ! Robot Parameters Structure
    RECORD robot_param
        speeddata speed;
        zonedata zone;
        num obj_offset;
        num wait_sgT;
    ENDRECORD
    ! Robot Control Structure
    RECORD robot_ctrl_str
        num actual_state;
        syncident syncT;
        robot_param r_param; 
    ENDRECORD
    
    ! Call Main Structure
    VAR robot_ctrl_str r_str;
    
    ! Main waypoints (targets) for robot control
    CONST robtarget Target_Home_R:=[[422.759230972,-355.853,108.084570746],[-0.000000043,1,0.000002488,-0.00000597],[-1,1,0,0],[107.880618475,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Conv_R:=[[478.310998567,-214.599983839,37.108961198],[-0.000000043,1,0.000002488,-0.00000597],[-1,0,2,0],[107.880618475,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Conv_R_2:=[[478.310998567,-214.599983839,117.25961198],[-0.000000043,1,0.000002488,-0.00000597],[-1,0,2,0],[107.880618475,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_R:=[[308.311000432,-43.109002,175.400004791],[0.707106812,-0.707106751,0.000002462,0.000005981],[-1,1,1,0],[107.880618995,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! PERS Variables -> Communication between multiple Tasks (T_ROB_L, T_ROB_R)
    PERS bool sync_anim_move;
    
    ! Initialization Tasks for synchronization
    PERS tasks Task_list{2} := [ ["T_ROB_L"], ["T_ROB_R"] ];
    
    ! Declaration of meters
    VAR num iteration_count:=0;
    VAR num iteration_count1:=0;
    
    ! Variables for offset
    VAR num x := 0;
    VAR num y := 0;

 ! VAR socketdev serverSocket;
    ! VAR socketdev clientSocket;
    ! VAR string data;
    
    ! ################################## ABB YUMI (RIGHT ARM) - Main Cycle ################################## !
    PROC main()
                    MVM_R;              
    ENDPROC
        
PROC MVM_R()
   TEST r_str.actual_state
            CASE 0:
                ! ******************** INITIALIZATION STATE ******************** !
                ! Initialize the parameters
                INIT_PARAM;
                ! Restore the environment
                RESET_ENV;
                ! Move -> Home position

                MoveJ Target_Home_R,v200,fine,Servo\WObj:=wobj0;

                IF sync_anim_move=TRUE THEN 
                    ! Wait for the digital input from the T_ROB_L
                    !waits for the digital output DO_INPOS_ROB_L to be active for 1 second.
                     WaitDO DO_INPOS_ROB_R, 1;
                     
                ENDIF
                r_str.actual_state := 1;

            CASE 1:
                !MoveL Offs(Target_Conv_R, x, y, 0),r_str.r_param.speed,fine,Servo\WObj:=wobj0; 
                MoveL Offs(Target_Conv_R_2, x, y, 0),r_str.r_param.speed,fine,Servo\WObj:=wobj0; 
                MoveJ Offs(Target_Sync_R, 0, 0, 0),r_str.r_param.speed,fine,Servo\WObj:=wobj0;      
                r_str.actual_state := 2;
                
            CASE 2:
                ! ******************** WAIT STATE (SYNC. Move) ******************** !
                ! Pulse signal for Robot Arm (L) -> motion command (The robot R is in position)
                PulseDO DO_INPOS_ROB_L;               
                IF sync_anim_move = TRUE THEN
                    ! Change state -> {"Piece validated"}
                    r_str.actual_state := 3;
                ELSEIF sync_anim_move = FALSE THEN  
                   ! Change state -> {"Piece not validated"}
                    r_str.actual_state := 4;
                    ! Wait for tasks to sync.
                    WaitSyncTask r_str.syncT, Task_list;
                    ! Turn On -> Synchronization
                    SyncMoveOn r_str.syncT, Task_list;
                ENDIF
            CASE 3:
                PulseDO DO_CLOSE_GR;
                PulseDO DO_ATTACH_OBJ_L;
                r_str.actual_state := 10;

            CASE 4:
                ! ******************** SYNC. MOVE STATE ******************** !
                SYNC_MOVE_FCE 1, r_str.r_param.speed, r_str.r_param.zone, 100, 50;
                SyncMoveOff r_str.syncT;
                MoveJ Target_Home_R,v200,fine,Servo\WObj:=wobj0;
                !MoveL Offs(Target_Home_R, x, y, 0),r_str.r_param.speed,fine,Servo\WObj:=wobj0; 

                r_str.actual_state := 0;
            CASE 10:
                MoveL Offs(Target_Conv_R_2, x, y, 0),r_str.r_param.speed,fine,Servo\WObj:=wobj0; 
                MoveL Offs(Target_Conv_R, x, y, 0),r_str.r_param.speed,fine,Servo\WObj:=wobj0; 
                ! Change state -> {empty}
            
                r_str.actual_state := 100;
            CASE 100:
             
                PulseDO DO_INPOS_ROB_R;
                iteration_count := iteration_count + 1;
                y :=  y - 35;
                    IF iteration_count < 5 THEN
                    ! Reset for the next iterations
                    r_str.actual_state := 0;  
                ELSE
                    ! Cycle completed after 5 iterations
                    r_str.actual_state := 101;  
                ENDIF
            CASE 101:
                iteration_count1 := iteration_count1 + 1;
                IF iteration_count1 < 3 THEN   
                    ! Reset for the next Cycle
                    r_str.actual_state := 0; 
                    iteration_count :=0;
                     y := 0;
                    x:=x-40;
           
                ELSE                    
                    r_str.actual_state := 1011;  ! Fin du cycle après 4 itérations
                ENDIF
            CASE 1011:
                                ! ******************** EMPTY STATE ******************** !
                                !End of process
        ENDTEST
        
ENDPROC
  
    ! ################################## SYNC. MOVE FUNCTION ################################## !
    PROC SYNC_MOVE_FCE(num sync_id, speeddata speed, zonedata zone, num offs_param_y, num offs_param_z)
        MoveL Offs(Target_Sync_R, 0, -25, 0),\ID:=sync_id, v50,fine,Servo\WObj:=wobj0;       
        WaitTime r_str.r_param.wait_sgT;
        MoveL Offs(Target_Sync_R, 0, 0, 0),\ID:=sync_id, speed,fine,Servo\WObj:=wobj0;       
        WaitTime r_str.r_param.wait_sgT;
        MoveL Offs(Target_Sync_R, 0, 0, -104),\ID:=sync_id, speed,fine,Servo\WObj:=wobj0;       
        PulseDO DO_OPEN_GL;
        ! Signal -> Detach the object: The location of the object is on the conveyor.
        PulseDO DO_DETACH_OBJ_L;
        MoveL Offs(Target_Sync_R, 0, 0, 0),\ID:=sync_id, speed,fine,Servo\WObj:=wobj0;       
        WaitTime r_str.r_param.wait_sgT;

        iteration_count := iteration_count + 1;
        y :=  y - 35;
        
    ENDPROC
    ! ################################## INIT PARAMETERS ################################## !
    PROC INIT_PARAM()
        ! Intitialization parameters of the robot
        ! Speed
        r_str.r_param.speed := [200, 200, 200, 200];
        ! Zone
        r_str.r_param.zone  := z50;
        ! Object offset
        r_str.r_param.obj_offset := 100;
        ! Wait time (Smart Gripper) -> [seconds]
        r_str.r_param.wait_sgT := 0.5;
        
    ENDPROC
    ! ################################## RESET ENVIRONMENT ################################## !
    PROC RESET_ENV()
        ! Reset DO (Digital Output)
        ! Detacher {Environment}
        PulseDO DO_RESET_ENV; 
        ! Detacher {Control Object}
        PulseDO DO_DETACH_OBJ_R;
        ! Smart Gripper -> Release (Home Position)
        PulseDO DO_OPEN_GR;
    ENDPROC
    ! ################################## TEST TARGETS ################################## !
    PROC Path_20()
        MoveJ Target_Home_R,v200,fine,Servo\WObj:=wobj0;
        MoveL Target_Conv_R,v200,fine,Servo\WObj:=wobj0;
        MoveL Target_Sync_R,v200,fine,Servo\WObj:=wobj0;
    ENDPROC
ENDMODULE