MODULE SocketServerII

    ! Socket communication variables
    VAR socketdev server_socket;
    VAR socketdev client_socket;
    VAR bool client_connected := FALSE;

    ! Communication variables
    VAR string received_msg;
    VAR string response_msg;

    ! Robot position variables
    VAR jointtarget current_joints;
    VAR jointtarget target_joints;
    VAR robtarget target_cart;

    ! Configuration
    CONST num SERVER_PORT := 5000;

    ! Speed and zone data
    VAR speeddata sock_speed := [50, 50, 50, 50];
    VAR zonedata move_zone := fine;

    PROC SocketMain()
        TPWrite "=== Socket Server II Starting ===";
        TPWrite "Port: 5000";

        ! Close any existing sockets from previous run
        CloseAllSockets;

        ! Reset connection state
        client_connected := FALSE;

        ! Create and bind server socket
        SocketCreate server_socket;
        SocketBind server_socket, "192.168.125.1", SERVER_PORT;
        SocketListen server_socket;

        TPWrite "Waiting for connection...";

        MainLoop;

    ERROR
        IF ERRNO = ERR_SOCK_TIMEOUT THEN
            ! Timeout - just retry
            TRYNEXT;
        ELSE
            TPWrite "Main Error: " + ValToStr(ERRNO);
            SocketClose client_socket;
            SocketClose server_socket;
            ! Restart the socket server
            RETRY;
        ENDIF
    ENDPROC

    PROC MainLoop()
        VAR num cmd_id;
        VAR string cmd_part;

        WHILE TRUE DO
            ! Wait for client
            IF client_connected = FALSE THEN
                ! Close any existing client socket first (ignore errors)
                CloseClientSafe;
                SocketAccept server_socket, client_socket;
                client_connected := TRUE;
                TPWrite "Client connected!";
            ENDIF

            ! Receive command
            SocketReceive client_socket \Str:=received_msg;

            ! Use received message directly - check command by prefix
            cmd_part := received_msg;

            ! Default response
            response_msg := "ERROR:Unknown command";

            ! Process command by comparing first N characters
            IF StrLen(cmd_part) >= 4 AND StrPart(cmd_part, 1, 4) = "PING" THEN
                response_msg := "PONG";

            ELSEIF StrLen(cmd_part) >= 6 AND StrPart(cmd_part, 1, 6) = "GETPOS" THEN
                current_joints := CJointT();
                response_msg := "POS:"
                    + ValToStr(current_joints.robax.rax_1) + " "
                    + ValToStr(current_joints.robax.rax_2) + " "
                    + ValToStr(current_joints.robax.rax_3) + " "
                    + ValToStr(current_joints.robax.rax_4) + " "
                    + ValToStr(current_joints.robax.rax_5) + " "
                    + ValToStr(current_joints.robax.rax_6);

            ELSEIF StrLen(cmd_part) >= 7 AND StrPart(cmd_part, 1, 7) = "GETCART" THEN
                ! Get current Cartesian position
                GetCartPos;

            ELSEIF StrLen(cmd_part) >= 4 AND StrPart(cmd_part, 1, 4) = "QUIT" THEN
                response_msg := "OK:Goodbye";
                SocketSend client_socket \Str:=response_msg;
                client_connected := FALSE;
                SocketClose client_socket;

            ELSEIF StrLen(cmd_part) >= 5 AND StrPart(cmd_part, 1, 5) = "MOVEJ" THEN
                ! Parse MOVEJ command (joint move)
                ParseAndMove cmd_part;

            ELSEIF StrLen(cmd_part) >= 5 AND StrPart(cmd_part, 1, 5) = "MOVEL" THEN
                ! Parse MOVEL command (linear Cartesian move)
                ParseAndMoveL cmd_part;

            ELSEIF StrLen(cmd_part) >= 5 AND StrPart(cmd_part, 1, 5) = "MOVEZ" THEN
                ! Parse MOVEZ command (linear move, change Z only relative to current)
                ParseAndMoveZ cmd_part;

            ELSEIF StrLen(cmd_part) >= 5 AND StrPart(cmd_part, 1, 5) = "SPEED" THEN
                ! Parse SPEED command
                ParseSpeed cmd_part;

            ELSEIF StrLen(cmd_part) >= 4 AND StrPart(cmd_part, 1, 4) = "GRIP" THEN
                ! Close gripper
                CloseGripper;

            ELSEIF StrLen(cmd_part) >= 7 AND StrPart(cmd_part, 1, 7) = "RELEASE" THEN
                ! Open gripper
                OpenGripper;

            ENDIF

            ! Send response (if still connected)
            IF client_connected THEN
                SocketSend client_socket \Str:=response_msg;
            ENDIF

        ENDWHILE

    ERROR
        IF ERRNO = ERR_SOCK_CLOSED THEN
            TPWrite "Client disconnected";
            client_connected := FALSE;
            TRYNEXT;
        ELSEIF ERRNO = ERR_SOCK_TIMEOUT THEN
            ! Timeout is normal when waiting - just retry
            TRYNEXT;
        ELSE
            TPWrite "Loop Error: " + ValToStr(ERRNO);
            client_connected := FALSE;
            TRYNEXT;
        ENDIF
    ENDPROC

    PROC ParseAndMove(string cmd)
        VAR num j1; VAR num j2; VAR num j3;
        VAR num j4; VAR num j5; VAR num j6;
        VAR num pos;
        VAR num next_pos;
        VAR string val_str;
        VAR bool ok;

        ! cmd format: "MOVEJ j1 j2 j3 j4 j5 j6"
        pos := 7;  ! Start after "MOVEJ "

        ! Parse j1
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, j1);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse j1 failed";
            RETURN;
        ENDIF

        ! Parse j2
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, j2);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse j2 failed";
            RETURN;
        ENDIF

        ! Parse j3
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, j3);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse j3 failed";
            RETURN;
        ENDIF

        ! Parse j4
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, j4);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse j4 failed";
            RETURN;
        ENDIF

        ! Parse j5
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, j5);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse j5 failed";
            RETURN;
        ENDIF

        ! Parse j6 (last value)
        val_str := StrPart(cmd, pos, StrLen(cmd) - pos + 1);
        ok := StrToVal(val_str, j6);

        IF NOT ok THEN
            response_msg := "ERROR:Parse j6 failed";
            RETURN;
        ENDIF

        ! Execute move
        TPWrite "Moving to: " + ValToStr(j1) + "," + ValToStr(j2) + "...";
        target_joints := [[j1, j2, j3, j4, j5, j6], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
        MoveAbsJ target_joints, sock_speed, move_zone, tool0;
        response_msg := "OK:Move complete";

    ERROR
        response_msg := "ERROR:Move failed - " + ValToStr(ERRNO);
        TRYNEXT;
    ENDPROC

    PROC ParseAndMoveL(string cmd)
        VAR num x; VAR num y; VAR num z;
        VAR num q0; VAR num q1; VAR num q2; VAR num q3;
        VAR num pos;
        VAR num next_pos;
        VAR string val_str;
        VAR bool ok;
        VAR robtarget current_cart;

        ! cmd format: "MOVEL x y z q0 q1 q2 q3"
        pos := 7;  ! Start after "MOVEL "

        ! Parse x
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, x);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse x failed";
            RETURN;
        ENDIF

        ! Parse y
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, y);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse y failed";
            RETURN;
        ENDIF

        ! Parse z
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, z);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse z failed";
            RETURN;
        ENDIF

        ! Parse q0
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, q0);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse q0 failed";
            RETURN;
        ENDIF

        ! Parse q1
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, q1);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse q1 failed";
            RETURN;
        ENDIF

        ! Parse q2
        next_pos := StrFind(cmd, pos, " ");
        IF next_pos > pos THEN
            val_str := StrPart(cmd, pos, next_pos - pos);
            ok := StrToVal(val_str, q2);
            pos := next_pos + 1;
        ELSE
            response_msg := "ERROR:Parse q2 failed";
            RETURN;
        ENDIF

        ! Parse q3 (last value)
        val_str := StrPart(cmd, pos, StrLen(cmd) - pos + 1);
        ok := StrToVal(val_str, q3);

        IF NOT ok THEN
            response_msg := "ERROR:Parse q3 failed";
            RETURN;
        ENDIF

        ! Get current position for config data
        current_cart := CRobT(\Tool:=tool0);

        ! Build target robtarget
        target_cart.trans.x := x;
        target_cart.trans.y := y;
        target_cart.trans.z := z;
        target_cart.rot.q1 := q0;
        target_cart.rot.q2 := q1;
        target_cart.rot.q3 := q2;
        target_cart.rot.q4 := q3;
        target_cart.robconf := current_cart.robconf;
        target_cart.extax := [9E9, 9E9, 9E9, 9E9, 9E9, 9E9];

        ! Disable configuration checking for flexibility
        ConfL \Off;

        ! Enable singularity handling (wrist singularity interpolation)
        SingArea \Wrist;

        ! Execute linear move
        TPWrite "MoveL to: " + ValToStr(x) + "," + ValToStr(y) + "," + ValToStr(z);
        MoveL target_cart, sock_speed, z1, tool0;
        response_msg := "OK:MoveL complete";

    ERROR
        response_msg := "ERROR:MoveL failed - " + ValToStr(ERRNO);
        TRYNEXT;
    ENDPROC

    PROC ParseAndMoveZ(string cmd)
        VAR num delta_z;
        VAR string val_str;
        VAR bool ok;
        VAR robtarget current_cart;
        VAR robtarget target_cart;

        ! cmd format: "MOVEZ delta_z" (delta in mm, positive=up, negative=down)
        val_str := StrPart(cmd, 7, StrLen(cmd) - 6);
        ok := StrToVal(val_str, delta_z);

        IF NOT ok THEN
            response_msg := "ERROR:Parse delta_z failed";
            RETURN;
        ENDIF

        ! Get current position (keeps X, Y, orientation)
        current_cart := CRobT(\Tool:=tool0);

        ! Build target: same as current, but Z + delta
        target_cart := current_cart;
        target_cart.trans.z := current_cart.trans.z + delta_z;

        ! Disable configuration checking for flexibility
        ConfL \Off;

        ! Execute linear move (use fine for precise positioning)
        TPWrite "MoveZ: delta=" + ValToStr(delta_z) + " -> Z=" + ValToStr(target_cart.trans.z);
        MoveL target_cart, sock_speed, fine, tool0;
        response_msg := "OK:MoveZ complete";

    ERROR
        response_msg := "ERROR:MoveZ failed - " + ValToStr(ERRNO);
        TRYNEXT;
    ENDPROC

    PROC GetCartPos()
        VAR robtarget current_cart;
        current_cart := CRobT(\Tool:=tool0);
        response_msg := "CART:"
            + ValToStr(current_cart.trans.x) + " "
            + ValToStr(current_cart.trans.y) + " "
            + ValToStr(current_cart.trans.z) + " "
            + ValToStr(current_cart.rot.q1) + " "
            + ValToStr(current_cart.rot.q2) + " "
            + ValToStr(current_cart.rot.q3) + " "
            + ValToStr(current_cart.rot.q4);
    ENDPROC

    PROC ParseSpeed(string cmd)
        VAR num speed_val;
        VAR string val_str;
        VAR bool ok;

        ! cmd format: "SPEED value"
        val_str := StrPart(cmd, 7, StrLen(cmd) - 6);
        ok := StrToVal(val_str, speed_val);

        IF ok AND speed_val > 0 AND speed_val <= 500 THEN
            sock_speed := [speed_val, 50, 50, 50];
            response_msg := "OK:Speed set to " + ValToStr(speed_val);
        ELSE
            response_msg := "ERROR:Invalid speed";
        ENDIF
    ENDPROC

    PROC CloseClientSafe()
        SocketClose client_socket;
    ERROR
        ! Ignore errors when closing (socket might not exist)
        TRYNEXT;
    ENDPROC

    PROC CloseAllSockets()
        ! Close client socket first
        SocketClose client_socket;
        ! Then close server socket
        SocketClose server_socket;
    ERROR
        ! Ignore all errors - sockets might not exist
        TRYNEXT;
    ENDPROC

    PROC CloseGripper()
        Reset Gripper_Open;
        Set Gripper_Close;
        WaitTime 0.5;
        response_msg := "OK:Gripper closed";
    ERROR
        response_msg := "ERROR:Gripper close failed - " + ValToStr(ERRNO);
        TRYNEXT;
    ENDPROC

    PROC OpenGripper()
        Reset Gripper_Close;
        Set Gripper_Open;
        WaitTime 0.5;
        response_msg := "OK:Gripper opened";
    ERROR
        response_msg := "ERROR:Gripper open failed - " + ValToStr(ERRNO);
        TRYNEXT;
    ENDPROC

    ! Main entry point - required for AUTO mode execution
    PROC main()
        SocketMain;
    ENDPROC

ENDMODULE
