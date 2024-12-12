package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class WristClaw {
    OpMode _opMode;
    Servo _wrist;
    Servo _claw;
    public WristClaw (OpMode opMode)
    {
        _opMode = opMode;
        _wrist = _opMode.hardwareMap.tryGet(Servo.class, "wrist");
        _claw = _opMode.hardwareMap.tryGet(Servo.class, "claw");
    }
    //-----------------------------------------
    //Variable Storage:
    double openPosition = 0.57;
    double closePosition = 0.2;
    double upPosition = 0.8;
    double flipPosition = 0.45;
    double downPosition = 0.42;
    double dumpPosition = 0.4;
    double centerPosition = 0.5;
    double wristInit = 0.755;
    //-----------------------------------------

    public void OpenClaw() {ClawCustom(openPosition);}
    public void CloseClaw() {ClawCustom(closePosition);}
    public void WristUp() {WristCustom(upPosition);}
    public void WristFlip() {WristCustom(flipPosition);}
    public void WristDown() {WristCustom(downPosition);}
    public void WristDump() {WristCustom(dumpPosition);}
    public void WristCenter() {WristCustom(centerPosition);}
    public void WristInit() {WristCustom(wristInit);}
    public void WristCustom(double position)
    {
        if (_wrist == null)
        {
            _opMode.telemetry.addLine("Wrist servo not found!");
        } else {
            _wrist.setPosition(position);
        }

    }
    public void ClawCustom(double position)
    {
        if (_claw == null)
        {
            _opMode.telemetry.addLine("Claw Servo not found!");
        } else {
            _claw.setPosition(position);
        }
    }
}
