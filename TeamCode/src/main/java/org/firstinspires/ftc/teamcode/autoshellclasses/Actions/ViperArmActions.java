package org.firstinspires.ftc.teamcode.autoshellclasses.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BBcode.Arm;
import org.firstinspires.ftc.teamcode.BBcode.Viper;

public class ViperArmActions {
    private final Arm arm;
    private final Viper viper;

    public ViperArmActions(OpMode opMode) {
        arm = new Arm(opMode);
        viper = new Viper(opMode);
    }


    public class basket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.MoveToHighBasket();
            viper.ExtendFull(1);
            return false;
        }
    }
    public Action Basket() {
        return new ViperArmActions.basket();
    }

    public class floor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.MoveToHome();
            viper.ExtendShort(1);
            return false;
        }
    }
    public Action Floor() {
        return new ViperArmActions.floor();
    }
}
