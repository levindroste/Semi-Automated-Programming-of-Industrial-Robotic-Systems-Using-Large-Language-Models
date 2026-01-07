MODULE GeneratedTrajectory

    ! ================================================
    ! Auto-generated RAPID code from MoveIt trajectory
    ! ================================================

    ! Speed configuration
    CONST speeddata motion_speed := [50,50,50,50];

    ! Target positions
    CONST robtarget pTarget1 := [[374.00,0.00,630.00],[0.707107,0.000000,0.707107,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget2 := [[385.64,0.00,616.74],[0.694658,0.000000,0.719340,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget3 := [[396.81,0.00,603.08],[0.681998,0.000000,0.731354,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget4 := [[407.49,0.00,589.04],[0.669131,0.000000,0.743145,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget5 := [[417.68,0.00,574.64],[0.656059,0.000000,0.754710,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget6 := [[427.36,0.00,559.89],[0.642788,0.000000,0.766044,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget7 := [[436.52,0.00,544.81],[0.629320,0.000000,0.777146,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget8 := [[445.14,0.00,529.42],[0.615661,0.000000,0.788011,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget9 := [[453.23,0.00,513.74],[0.601815,0.000000,0.798636,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget10 := [[460.76,0.00,497.79],[0.587785,0.000000,0.809017,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget11 := [[467.73,0.00,481.58],[0.573576,0.000000,0.819152,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget12 := [[474.13,0.00,465.14],[0.559193,0.000000,0.829038,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget13 := [[479.96,0.00,448.49],[0.544639,0.000000,0.838671,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget14 := [[485.20,0.00,431.64],[0.529919,0.000000,0.848048,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget15 := [[489.84,0.00,414.62],[0.515038,0.000000,0.857167,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget16 := [[493.89,0.00,397.45],[0.500000,0.000000,0.866025,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget17 := [[497.34,0.00,380.15],[0.484810,0.000000,0.874620,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget pTarget18 := [[500.19,0.00,362.73],[0.469472,0.000000,0.882948,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget jTarget19 := [[0.0000,36.0000,0.0000,0.0000,0.0000,0.0000],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST jointtarget jTarget20 := [[0.0000,38.0000,0.0000,0.0000,0.0000,0.0000],[9E9,9E9,9E9,9E9,9E9,9E9]];

    PROC main()
        ! Initialize
        ConfJ \Off;
        ConfL \Off;

        ! Execute trajectory
        ! Linear segment 1
        MoveL pTarget1, v50, z1, tool0;
        MoveL pTarget2, v50, z1, tool0;
        MoveL pTarget3, v50, z1, tool0;
        ! Linear segment 2
        MoveL pTarget4, v50, z1, tool0;
        MoveL pTarget5, v50, z1, tool0;
        MoveL pTarget6, v50, z1, tool0;
        ! Linear segment 3
        MoveL pTarget7, v50, z1, tool0;
        MoveL pTarget8, v50, z1, tool0;
        MoveL pTarget9, v50, z1, tool0;
        ! Linear segment 4
        MoveL pTarget10, v50, z1, tool0;
        MoveL pTarget11, v50, z1, tool0;
        MoveL pTarget12, v50, z1, tool0;
        ! Linear segment 5
        MoveL pTarget13, v50, z1, tool0;
        MoveL pTarget14, v50, z1, tool0;
        MoveL pTarget15, v50, z1, tool0;
        ! Linear segment 6
        MoveL pTarget16, v50, z1, tool0;
        MoveL pTarget17, v50, z1, tool0;
        MoveL pTarget18, v50, z1, tool0;
        ! Joint movement
        MoveAbsJ jTarget19, v50, z1, tool0;
        MoveAbsJ jTarget20, v50, z1, tool0;

        ! Done
        Stop;
    ENDPROC

ENDMODULE