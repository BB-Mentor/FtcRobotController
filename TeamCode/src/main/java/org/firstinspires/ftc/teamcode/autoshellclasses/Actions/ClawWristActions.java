package org.firstinspires.ftc.teamcode.autoshellclasses.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BBcode.WristClaw;

public class ClawWristActions {
    private final WristClaw wristClaw;

    public ClawWristActions(OpMode opMode) {
        wristClaw = new WristClaw(opMode);
    }

    public class OpenClawAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristClaw.OpenClaw();
            return false;
        }
    }
    public Action OpenClaw() {
        return new ClawWristActions.OpenClawAction();
    }

    public class CloseClawAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristClaw.CloseClaw();
            return false;
        }
    }
    public Action CloseClaw() {
        return new ClawWristActions.CloseClawAction();
    }

    public class WristDownAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristClaw.MoveDown();
            return false;
        }
    }
    public Action WristDown() {
        return new ClawWristActions.WristDownAction();
    }

    public class wristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristClaw.MoveUp();
            return false;
        }
    }
    public Action WristUp() {
        return new ClawWristActions.wristUp();
    }
}
