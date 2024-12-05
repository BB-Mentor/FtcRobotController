package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmActions {
    OpMode _OpMode;
    DcMotorEx _ArmMotor;

    //constructor
    public ArmActions(OpMode opmode) {
        _OpMode = opmode;
        _ArmMotor = _OpMode.hardwareMap.tryGet(DcMotorEx.class, "armMotor");
    }
    //----------------------------------------------------------------------------------------------
    //Arm Positions
    final int homePosition = 0;
    final int clearancePosition = 24;
    final int highBasketPosition = 80;

    //----------------------------------------------------------------------------------------------
    //Actions
    public Action MoveToHome() {return new MoveArmToAction(homePosition, 1);}
    public Action MoveToClearance() {return new MoveArmToAction(clearancePosition, 1);}
    public Action MoveToHighBasket() {return new MoveArmToAction(highBasketPosition, 1);}

    //----------------------------------------------------------------------------------------------
    //NonAction functions
    //for RR Action incompatible uses
    public void MoveToHomeNonAction() {
        MoveArmTo(homePosition, 1);}
    public void MoveToClearanceNonAction() {
        MoveArmTo(clearancePosition, 1);}
    public void MoveToHighBasketNonAction() {
        MoveArmTo(highBasketPosition, 1);}

    //----------------------------------------------------------------------------------------------
    //Base Action
    //can input any angle and power
    private class MoveArmToAction implements Action {
        double _Angle;
        double _Power;
        public MoveArmToAction(double angle, double power) {_Angle = angle; _Power = power;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            MoveArmTo(_Angle, _Power);
            return false;
        }
    }
    private void MoveArmTo(double angle, double power) {
        if (_ArmMotor == null) {
            _OpMode.telemetry.addLine("Arm motor not found!");
        }
        else {
            double ticksPerDegree = 7125.0/360.0;
            int angleTicks = (int)(angle * ticksPerDegree);
            _ArmMotor.setDirection(DcMotor.Direction.REVERSE);
            _ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _ArmMotor.setTargetPosition(angleTicks);
            _ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _ArmMotor.setPower(power);
        }
    }
}