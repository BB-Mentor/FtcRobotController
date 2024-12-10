package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    OpMode _OpMode;
    Servo _Wrist;

    //constructor
    public Wrist(OpMode opmode) {
        _OpMode = opmode;
        _Wrist = _OpMode.hardwareMap.tryGet(Servo.class, "wrist");
    }
    //----------------------------------------------------------------------------------------------
    //Wrist Positions
    double upPosition = 0.855;
    double downPosition = 0.4725;
    double dumpPosition = 0.45;

    //----------------------------------------------------------------------------------------------
    //Actions
    public Action MoveToUp() {return new MoveWristToAction(upPosition);}
    public Action MoveToDown() {return new MoveWristToAction(downPosition);}
    public Action MoveToBasket() {return new MoveWristToAction(dumpPosition);}

    //----------------------------------------------------------------------------------------------
    //NonAction functions
    //for RR Action incompatible uses
    public void MoveToUpNonAction() {MoveWristTo(upPosition);}
    public void MoveToDownNonAction() {MoveWristTo(downPosition);}
    public void MoveToBasketNonAction() {MoveWristTo(dumpPosition);}

    //----------------------------------------------------------------------------------------------
    //Base Action
    //can input any servo position
    private class MoveWristToAction implements Action {
        double _Position;
        public MoveWristToAction(double position) {
            _Position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            MoveWristTo(_Position);
            return false;
        }

    }
    private void MoveWristTo(double position) {
        if (_Wrist == null) {
            _OpMode.telemetry.addLine("Wrist Servo not found!");
        }
        else {
            _Wrist.setPosition(position);
        }
    }
}