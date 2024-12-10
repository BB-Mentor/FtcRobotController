package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    OpMode _OpMode;
    Servo _Claw;

    //constructor
    public Claw(OpMode opmode) {
        _OpMode = opmode;
        _Claw = _OpMode.hardwareMap.tryGet(Servo.class, "claw");
    }
    //----------------------------------------------------------------------------------------------
    //Claw Positions
    double openPosition = 0.57;
    double closePosition = 0.2;

    //----------------------------------------------------------------------------------------------
    //Actions
    public Action Open() {return new MoveClawToAction(openPosition);}
    public Action Close() {return new MoveClawToAction(closePosition);}

    //----------------------------------------------------------------------------------------------
    //NonAction functions
    //for RR Action incompatible uses
    public void OpenNonAction() {MoveClawTo(openPosition);}
    public void CloseNonAction() {MoveClawTo(openPosition);}

    //----------------------------------------------------------------------------------------------
    //Base Action
    //can input any servo position
    private class MoveClawToAction implements Action {
        double _Position;
        public MoveClawToAction(double position) {
            _Position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            MoveClawTo(_Position);
            return false;
        }
    }
    private void MoveClawTo(double position) {
        if (_Claw == null) {
            _OpMode.telemetry.addLine("Claw Servo not found!");
        }
        else {
            _Claw.setPosition(position);
        }
    }
}