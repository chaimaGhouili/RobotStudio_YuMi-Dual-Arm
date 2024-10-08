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
    
    ! targets for robot control
    CONST robtarget Home:=[[518.310945519,315.399982287,141.76701641],[0.000000052,1,-0.000000119,-0.000000303],[1,0,-2,0],[-134.999969429,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Conv_L:=[[518.311,315.4,37.109],[0,1,0,0],[1,0,-2,0],[-134.999969429,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_L:=[[308.301998612,31.891000222,175.422999954],[0.707106744,0.707106818,0.00000013,-0.000000298],[1,-1,-1,0],[-134.999973742,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    ! PERS Variables -> Communication between multiple Tasks (T_ROB_L, T_ROB_R)
    pers bool sync_anim_move;
    ! Initialization Tasks for synchronization
    PERS tasks Task_list{2} := [ ["T_ROB_L"], ["T_ROB_R"] ];
    
    ! Variables for offset
    VAR num x := 0;
    VAR num y := 0;
    
    ! Declaration of meters
    VAR num iteration_count:=0;
    VAR num iteration_count1:=0;
    
     VAR socketdev serverSocket;
     VAR socketdev clientSocket;
     VAR string data;
    

    ! ################################## ABB YUMI (LEFT ARM) - Main Cycle ################################## !
    PROC main()              
      ! Configuration du serveur socket
      
         SocketCreate serverSocket;
         SocketBind serverSocket, "127.0.0.1", 8080;
        SocketListen serverSocket;
         SocketAccept serverSocket, clientSocket, \Time:=WAIT_MAX;
    
      WHILE TRUE DO
             SocketReceive clientSocket\str:=data;
            ! V�rifie si les donn�es re�ues sont "TRUE" ou "FALSE"
            IF data = "TRUE" THEN
                 sync_anim_move := TRUE;
             ELSE
                 sync_anim_move := FALSE;
            ENDIF

         
             WaitTime 1;
         ENDWHILE

         ! Ferme la connexion
         SocketClose clientSocket;
        SocketClose serverSocket;
        MVM_L;     
    ENDPROC
       
    PROC MVM_L()
        TEST r_str.actual_state
        
            CASE 0:
                ! Initialize the parameters
                INIT_PARAM;
                ! Restore the environment
                RESET_ENV;
                ! Move -> Home position
                MoveL Offs(Home, x, y, 0),r_str.r_param.speed,r_str.r_param.zone,Servo\WObj:=wobj0;

               ! MoveJ Home,r_str.r_param.speed,fine,Servo\WObj:=wobj0;
                WaitTime r_str.r_param.wait_sgT;
                ! Change state 
                r_str.actual_state := 1;    
                
            CASE 1:               
                Place_OBJ;
                MoveL Offs(Home, x, y, 0),r_str.r_param.speed,r_str.r_param.zone,Servo\WObj:=wobj0;
                ! Move to the Sync. Position (with offset)
                MoveL Offs(Target_Sync_L, 0, r_str.r_param.obj_offset, 0),r_str.r_param.speed,r_str.r_param.zone,Servo\WObj:=wobj0;
                !move to test on camera 
                MoveL Offs(Target_Sync_L, 190, 0, 0),r_str.r_param.speed,r_str.r_param.zone,Servo\WObj:=wobj0;         
                WaitTime 2;                                                                  
                r_str.actual_state := 2;     
                
            CASE 2:
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
                  ! Move to the Position (with object attached)
                 MoveL Offs(Target_Sync_L, 0, 0, 0),r_str.r_param.speed,fine,Servo\WObj:=wobj0;
                 WaitTime r_str.r_param.wait_sgT;
                 ! Signal ->  configures a digital output with a pulse for the output 
                 PulseDO DO_INPOS_ROB_R;               
                r_str.actual_state := 5;

            CASE 4:
                ! ******************** SYNC. MOVE STATE ******************** !
                SYNC_MOVE_FCE 1, r_str.r_param.speed, r_str.r_param.zone, 100, 50;
                ! Turn Off -> Synchronization
                SyncMoveOff r_str.syncT;
                ! Change state -> {"Switch to another piece"}
                r_str.actual_state := 0;
          
            
               CASE 5:
               
               ! Signal -> Detach the object: The location of the object is on the conveyor.
               PulseDO DO_OPEN_GL;
               PulseDO DO_DETACH_OBJ_L;
               r_str.actual_state := 100;

          
              
            CASE 100:
                !   movement according to y
                WaitDO DO_INPOS_ROB_L, 1;
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
                !   movement according to x
                iteration_count1 := iteration_count1 + 1;

                IF iteration_count1 < 3 THEN
                    ! Reset for the next Cycle
                    r_str.actual_state := 0; 
                    iteration_count :=0;
                    y := 0;
                    x:=x-40;
                ELSE
                    r_str.actual_state := 1011;  
            ENDIF
            
            CASE 1011:
                                ! ******************** EMPTY STATE ******************** !
                                !End of process

        ENDTEST
    ENDPROC
    ! ################################## P&P FUNCTION (OBJ. HOLD. -> CONV) ################################## !
    PROC Place_OBJ ()
        !MoveJ Target_Conv_L,v200,fine,Servo\WObj:=wobj0;
        MoveJ Offs(Target_Conv_L, x, y, 0),v200,fine,Servo\WObj:=wobj0;
        PulseDO DO_CLOSE_GL;
        ! Signal -> Attach the object: The location of the object is on the object holder.
        PulseDO DO_ATTACH_OBJ_L;
        WaitTime r_str.r_param.wait_sgT;
       
        ENDPROC
    ! ################################## SYNC. MOVE FUNCTION ################################## !
    PROC SYNC_MOVE_FCE(num sync_id, speeddata speed, zonedata zone, num offs_param_y, num offs_param_z)
        MoveL Offs(Target_Sync_L, 0, 10, 0),\ID:=sync_id, speed,fine,Servo\WObj:=wobj0;       
        WaitTime r_str.r_param.wait_sgT;
        MoveL Offs(Target_Sync_L, 0, 0, 0),\ID:=sync_id, speed,fine,Servo\WObj:=wobj0;       
        WaitTime r_str.r_param.wait_sgT;
        MoveL Offs(Target_Sync_L, 0, 0, -104),\ID:=sync_id, speed,fine,Servo\WObj:=wobj0;       
        PulseDO DO_OPEN_GL;
        ! Signal -> Detach the object: The location of the object is on the conveyor.
        PulseDO DO_DETACH_OBJ_L;
        MoveL Offs(Target_Sync_L, 0, 0, 0),\ID:=sync_id, speed,fine,Servo\WObj:=wobj0;       

        ! Signal -> Move the position of the fingers (SyncePose): Gripp the object
        WaitTime r_str.r_param.wait_sgT;
        sync_anim_move := TRUE;

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
        ! Intitialization input parameters for the synchronization move
    ENDPROC
    ! ################################## RESET ENVIRONMENT ################################## !
    PROC RESET_ENV()
        ! Reset DO (Digital Output)
        ! Detacher {Environment}
        PulseDO DO_RESET_ENV; 
        ! Detacher {Control Object}
        PulseDO DO_DETACH_OBJ_L;
        ! Smart Gripper -> Release (Home Position)
        PulseDO DO_OPEN_GL;
    ENDPROC
    ! ################################## TEST TARGETS ################################## !
    PROC Path_10()
        MoveL Home,v1000,fine,Servo\WObj:=wobj0;
        MoveJ Target_Conv_L,v200,fine,Servo\WObj:=wobj0;
        MoveL Target_Sync_L,v200,fine,Servo\WObj:=wobj0;
    ENDPROC
  
    
ENDMODULE
