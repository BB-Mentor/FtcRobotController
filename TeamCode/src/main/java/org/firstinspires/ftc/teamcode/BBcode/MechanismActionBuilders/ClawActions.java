package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawActions {
    OpMode _Opmode;
    Servo _Claw;

    //constructor
    public ClawActions(OpMode opmode) {
        _Opmode = opmode;
        _Claw = _Opmode.hardwareMap.tryGet(Servo.class, "claw");
    }
    //----------------------------------------------------------------------------------------------
    //Claw Positions
    double openPosition = 0.57;
    double closePosition = 0.2;

    //----------------------------------------------------------------------------------------------
    //Actions
    public Action Open() {return new MoveClawTo(openPosition);}
    public Action Close() {return new MoveClawTo(closePosition);}

    //----------------------------------------------------------------------------------------------
    //Base Action
    //can input any servo position
    public class MoveClawTo implements Action {
        double _Position;
        public MoveClawTo(double position) {
            _Position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (_Claw == null) {
                _Opmode.telemetry.addLine("Claw Servo not found!");
            }
            else {
                _Claw.setPosition(_Position);
            }
            return false;
        }

    }
}
