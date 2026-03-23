MODULE TRobRAPID
!=======================================================================================================================
! Copyright (c) 2017, ABB Schweiz AG
! All rights reserved.
!
! Modified by Rui Martins (up202108756@edu.fe.up.pt) — FEUP/INESCTEC 2026
! Dissertation: "Collaborative Robotics for RCD Pre-Assembly"
!
! Changes vs. original ABB StateMachine AddIn TRobRAPID:
!   - routine_name_input: removed LOCAL so RWS can write to it
!   - Added: wp_array{18}, wp_count, wp_speed (global, writable via RWS)
!   - Added: runMoveAbsJMulti routine (multi-waypoint MoveAbsJ loop)
!   - Added: CASE "runMoveAbsJMulti" in attemptRoutine()
!=======================================================================================================================

    !---------------------------------------------------------
    ! Data that an external system can set via RWS
    !---------------------------------------------------------
    ! Input for specifying a RAPID routine to run.
    ! NOTE: NOT LOCAL — must be writable by RWS.
    VAR string routine_name_input := stEmpty;

    ! Inputs to predefined routines: runMoveJ, runMoveAbsJ
    LOCAL VAR speeddata   move_speed_input := [100, 10, 100, 10];
    LOCAL VAR robtarget   move_robtarget_input;
    LOCAL VAR jointtarget move_jointtarget_input;

    ! Inputs to runCallByVar
    LOCAL VAR string callbyvar_name_input := stEmpty;
    LOCAL VAR num callbyvar_num_input := 0;

    ! Inputs to runModuleLoad / runModuleUnload
    LOCAL VAR string module_file_path_input := stEmpty;

    !---------------------------------------------------------
    ! Multi-waypoint data (written by ROS 2 via RWS)
    ! NOTE: NOT LOCAL — must be writable by RWS.
    !---------------------------------------------------------
    VAR jointtarget wp_array{18};
    VAR num         wp_count  := 0;
    VAR speeddata   wp_speed  := [200, 100, 200, 100];

    !---------------------------------------------------------
    ! Program data
    !---------------------------------------------------------
    LOCAL VAR intnum intnum_run_rapid_routine;

    !---------------------------------------------------------
    ! Primary procedures
    !---------------------------------------------------------
    PROC initializeRAPIDModule()
        move_robtarget_input := CRobT();
        move_jointtarget_input := CJointT();

        IDelete intnum_run_rapid_routine;
        CONNECT intnum_run_rapid_routine WITH handleRunRAPIDRoutine;
        ISignalDI RUN_RAPID_ROUTINE, HIGH, intnum_run_rapid_routine;
    ENDPROC

    PROC runRAPIDRoutine()
        attemptRoutine;
        current_state := STATE_IDLE;
    ENDPROC

    !---------------------------------------------------------
    ! Traps
    !---------------------------------------------------------
    LOCAL TRAP handleRunRAPIDRoutine
        IF routine_name_input <> stEmpty THEN
            printRAPIDMessage "Attempting '" + routine_name_input + "'";
            attemptNonBlockingRoutine;

            IF routine_name_input <> stEmpty THEN
                IF current_state = STATE_IDLE THEN
                    current_state := STATE_RUN_RAPID_ROUTINE;
                    RAISE CHANGE_STATE;
                ENDIF
            ENDIF
        ENDIF

        ERROR (CHANGE_STATE)
            RAISE CHANGE_STATE;
    ENDTRAP

    !---------------------------------------------------------
    ! Auxiliary procedures
    !---------------------------------------------------------
    LOCAL PROC attemptRoutine()
        VAR bool found;
        found := attemptSystemRoutine(routine_name_input);

        IF NOT found THEN
            TEST routine_name_input
                CASE "runMoveToCalibrationPosition":
                    runMoveToCalibrationPosition;

                CASE "runMoveJ":
                    runMoveJ;

                CASE "runMoveAbsJ":
                    runMoveAbsJ;

                CASE "runMoveAbsJMulti":
                    runMoveAbsJMulti;

                CASE "runCallByVar":
                    runCallByVar;

                CASE "setLeadthroughOn":
                    setLeadthroughOn;

                CASE "setLeadthroughOff":
                    setLeadthroughOff;
            ENDTEST
        ENDIF

        routine_name_input := stEmpty;
    ENDPROC

    PROC runMoveToCalibrationPosition()
        current_tool := CTool();
        MoveAbsJ getCalibrationTarget(), v100, fine, current_tool \WObj:=base_wobj;

        ERROR
            printRAPIDMessage "'runMoveToCalibrationPosition' failed!";
    ENDPROC

    LOCAL PROC runMoveJ()
        current_tool := CTool();
        MoveJ move_robtarget_input, move_speed_input, fine, current_tool \WObj:=base_wobj;

        ERROR
            printRAPIDMessage "'runMoveJ' failed!";
    ENDPROC

    LOCAL PROC runMoveAbsJ()
        current_tool := CTool();
        MoveAbsJ move_jointtarget_input, move_speed_input, fine, current_tool \WObj:=base_wobj;

        ERROR
            printRAPIDMessage "'runMoveAbsJ' failed!";
    ENDPROC

    LOCAL PROC runMoveAbsJMulti()
        VAR num i;
        current_tool := CTool();
        FOR i FROM 1 TO wp_count DO
            IF i < wp_count THEN
                MoveAbsJ wp_array{i}, wp_speed, z5, current_tool \WObj:=base_wobj;
            ELSE
                MoveAbsJ wp_array{i}, wp_speed, fine, current_tool \WObj:=base_wobj;
            ENDIF
        ENDFOR

        ERROR
            printRAPIDMessage "'runMoveAbsJMulti' failed at wp " + NumToStr(i, 0);
    ENDPROC

    LOCAL PROC runCallByVar()
        CallByVar callbyvar_name_input, callbyvar_num_input;

        ERROR
            printRAPIDMessage "'runCallByVar' failed!";
            TRYNEXT;
    ENDPROC

    PROC setLeadthroughOn()
        SetLeadThrough \On;

        ERROR
            printRAPIDMessage "'setLeadthroughOn' failed!";
    ENDPROC

    PROC setLeadthroughOff()
        SetLeadThrough \Off;

        ERROR
            printRAPIDMessage "'setLeadthroughOff' failed!";
    ENDPROC

    !---------------------------------------------------------
    ! Auxiliary procedures (non-blocking)
    !---------------------------------------------------------
    LOCAL PROC attemptNonBlockingRoutine()
        VAR bool found;
        found := attemptNonBlockingSystemRoutine(routine_name_input);

        IF NOT found THEN
            TEST routine_name_input
                CASE "runModuleLoad":
                    runModuleLoad;

                CASE "runModuleUnload":
                    runModuleUnload;

                DEFAULT:
                    found := FALSE;
            ENDTEST
        ENDIF

        IF found THEN
            routine_name_input := stEmpty;
        ENDIF
    ENDPROC

    LOCAL PROC runModuleLoad()
        Load module_file_path_input;

        ERROR
            IF ERRNO <> ERR_LOADED THEN
                printRAPIDMessage "'runModuleLoad' failed!";
            ENDIF
            TRYNEXT;
    ENDPROC

    LOCAL PROC runModuleUnload()
        UnLoad module_file_path_input;

        ERROR
            printRAPIDMessage "'runModuleUnload' failed!";
            TRYNEXT;
    ENDPROC

ENDMODULE
