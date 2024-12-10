package org.firstinspires.ftc.teamcode.BBcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.Arm;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.Claw;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.Viper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.Wrist;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpRR")
public class TeleOpRR extends OpMode {
    //global vars
    private List<Action> runningActions = new ArrayList<>();
    Claw _Claw;
    Wrist _Wrist;
    Viper _Viper;
    Arm _Arm;
    MecanumDrivetrain _MecanumDrivetrain;

    //gamepad functions
    private void Gamepad1() {

    }
    private void Gamepad2() {
        //Extend to full length, limited to 18 inches at low angles
        //High Basket
       // if ((gamepad.left_trigger > 0 && gamepad.dpad_up) || (desiredViperState == MainTeleOp.ViperState.Dump && arm.get_armMotor().getCurrentPosition()<150) ) {
     //       arm.MoveToHighBasket();
     //       desiredViperState = MainTeleOp.ViperState.Dump;
      //      viper.ExtendFull(0.75);
      //      wristClaw.WristDump();
      //  }

    //    //brings arm down
    //    if ((gamepad.left_trigger > 0 && gamepad.dpad_down) || (desiredViperState == MainTeleOp.ViperState.Closed && arm.get_armMotor().getCurrentPosition()>1300) ) {
    //        wristClaw.WristUp();
    //        desiredViperState = MainTeleOp.ViperState.Closed;
    //        viper.ExtendShort(1);
    //        arm.MoveToHome();
    //    }

        //Open Claw
        if(gamepad2.b) {
            _Claw.Open();
        }

        //Close Claw
        if(gamepad2.x) {
            _Claw.Close();
        }

        //Move Wrist Up
        if(gamepad2.y) {
            _Wrist.MoveToUp();
        }

        //Move Wrist Down
        if(gamepad2.a) {
            _Wrist.MoveToDown();
        }
    }

    //----------------------------------------------------------------------------------------------
    @Override
    public void init() {
    //Initialization steps
        //Creates instance of MechanismControllers
        _Claw = new Claw(this);
        _Wrist = new Wrist(this);
        _Viper = new Viper(this);
        _Arm = new Arm(this);

        //Creates instance of drive code class
        _MecanumDrivetrain = new MecanumDrivetrain(this);
    }
    //----------------------------------------------------------------------------------------------
    @Override
    public void loop() {
        _MecanumDrivetrain.Drive();


        //KEEP AT BOTTOM OF LOOP
        //updates and runs RR actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            if (action.run(new TelemetryPacket())) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
    }
}
