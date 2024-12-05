package org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ViperActions {
    OpMode _OpMode;
    DcMotorEx _ViperMotor;

    //constructor
    public ViperActions(OpMode opmode) {
        _OpMode = opmode;
        _ViperMotor = _OpMode.hardwareMap.tryGet(DcMotorEx.class, "viperMotor");
    }
    //----------------------------------------------------------------------------------------------
    //Viper Positions
    int fullExtensionLength = 24;
    int halfExtensionLength = 9;
    int homeExtensionLength = 3;

    //----------------------------------------------------------------------------------------------
    //Actions
    public Action ExtendToFull() {return new ExtendViperToAction(fullExtensionLength, 1);}
    public Action ExtendToHalf() {return new ExtendViperToAction(halfExtensionLength, 1);}
    public Action ExtendToHome() {return new ExtendViperToAction(homeExtensionLength, 1);}

    //----------------------------------------------------------------------------------------------
    //NonAction functions
    //for RR Action incompatible uses
    public void ExtendToFullNonAction() {ExtendViperTo(fullExtensionLength, 1);}
    public void ExtendToHalfNonAction() {ExtendViperTo(halfExtensionLength, 1);}
    public void ExtendToHomeNonAction() {ExtendViperTo(halfExtensionLength, 1);}

    //----------------------------------------------------------------------------------------------
    //Base Action
    //can input any length and power
    private class ExtendViperToAction implements Action {
        double _ExtensionLength;
        double _Power;
        public ExtendViperToAction(double extensionLength, double power) {_ExtensionLength = extensionLength; _Power = power;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ExtendViperTo(_ExtensionLength, _Power);
            return false;
        }
    }
    private void ExtendViperTo(double extensionLength, double power) {
        if (_ViperMotor == null) {
            _OpMode.telemetry.addLine("Viper motor not found!");
        }
        else {
            double ticksPerInch = 537.7/4.625;
            int extensionLengthTicks = (int)(extensionLength * ticksPerInch);
            _ViperMotor.setDirection(DcMotor.Direction.REVERSE);
            _ViperMotor.setTargetPosition(extensionLengthTicks);
            _ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _ViperMotor.setPower(power);
        }
    }
}