package org.firstinspires.ftc.teamcode.BBcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.BlueBasket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Arm;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Viper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.WristClaw;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import java.util.Locale;

@TeleOp(name = "MainTeleopQ2")
public class MainTeleOpQ2 extends LinearOpMode{
    enum HighBasketState {
        Home,
        RisingArmSample,
        ViperExtendFull,
        WristDump,
        HighBasket,
        WristUp,
        ViperClosed,
        ViperRetractedShort,
        LoweringArm,
    }

    enum SpecimenClipState {
        Home,
        WristUp,
        RaiseArm,
        ViperExtend,
        SpecimenHang,
        WristDown,
        ViperExtendShort,
        ViperExtendClosed,
        ArmLowerToHome
    }

    enum SubmersiblePickupState {
        Home,
        WristUpIn,
        ExtendViperPickup,
        SubmersiblePickup,
        WristUpOut,
        ExtendViperClosed
    }


    HighBasketState highBasketState = HighBasketState.Home;
    SpecimenClipState specimenClipState = SpecimenClipState.Home;
    SubmersiblePickupState submersiblePickupState = SubmersiblePickupState.Home;

    ElapsedTime wristTimer = new ElapsedTime();

    final double wristFlipTime = 0.75;

    private void handleGamepad1 (Viper viper, WristClaw wristClaw) {
        //Specimen Pickup Position
        if (gamepad1.x) {
            wristClaw.WristSpecimenPickup();
        }
        //Inspection wrist setup
        if (gamepad1.b) {
            wristClaw.WristMid();
        }
    }

    private void handleGamepad2 (WristClaw wristClaw) {

        //Open Claw
        if(gamepad2.b) {
            telemetry.update();
            wristClaw.OpenClaw();
        }

        //Close Claw
        if(gamepad2.x) {
            telemetry.update();
            wristClaw.CloseClaw();
        }

        //Move Claw Up
        if(gamepad2.y) {
            wristClaw.WristUp();
        }

        //Move Claw Down
        if(gamepad2.a) {
            wristClaw.WristDown();
        }
    }

    GoBildaPinpointDriverRR odo; // Declare OpMode member for the Odometry Computer
    public double xOffset = -7.002384767061902; //RRTune, -6.5; measured
    public double yOffset = -1.2229245167313665;
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here

        odo = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        double encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;
        odo.setOffsets(DistanceUnit.MM.fromInches(xOffset), DistanceUnit.MM.fromInches(yOffset));
        odo.setEncoderResolution(encoderResolution);
        odo.setEncoderDirections(xDirection, yDirection);
        odo.resetPosAndIMU();

        TelemetryHelper telemetryHelper = new TelemetryHelper(this);
        //Allows for telemetry to be added to without clearing previous data. This allows setting up telemetry functions to be called in the loop or adding telemetry items within a function and not having it cleared on next loop
        telemetry.setAutoClear(false);
        //Init for the other classes this opmode pulls methods from
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
        Arm arm = new Arm(this, telemetryHelper);
        Viper viper = new Viper(this);
        WristClaw wristClaw = new WristClaw(this);
        arm.Reset();
        viper.StopAndResetEncoder();
        wristTimer.reset();

        //Call the function to initialize telemetry functions
//        telemetryHelper.initMotorTelemetry( viperMotor, "viperMotor");
        telemetryHelper.initGamepadTelemetry(gamepad1);
        telemetryHelper.initGamepadTelemetry(gamepad2);
        //Where the start button is clicked, put some starting commands after
        waitForStart();
        arm.MoveToHome();
        //Use the following line for measuring auto locations
      odo.setPosition(BlueBasket.pose_basket_init_old);
//        odo.setPosition(PoseStorage.currentPose);
        telemetry.addData("PositionRR", ()-> getPinpoint(odo.getPositionRR()));
        telemetry.addData("Position", ()-> getPinpoint(odo.getPosition()));
        while(opModeIsActive()){ //while loop for when program is active
            odo.update();

            //Drive code
            drivetrain.Drive();

            handleGamepad1(viper, wristClaw);
            handleGamepad2(wristClaw);

            switch (highBasketState) {
                case Home:
                    if (gamepad2.left_trigger > 0 && gamepad2.dpad_up) {
                        arm.MoveToHighBasket();
                        highBasketState = HighBasketState.RisingArmSample;
                    }
                    break;
                case RisingArmSample:
                    if (arm.getIsArmHighBasketPosition()) {
                        viper.ExtendFull(1);
                        highBasketState = HighBasketState.ViperExtendFull;
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        arm.MoveToHome();
                        highBasketState = HighBasketState.LoweringArm;
                    }
                    break;

                case ViperExtendFull:
                    if (viper.getIsViperExtendFull()) {
                        wristClaw.WristDump();
                        highBasketState = HighBasketState.WristDump;
                        wristTimer.reset();
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        viper.ExtendShort(1);
                        highBasketState = HighBasketState.ViperRetractedShort;
                    }
                    break;

                case WristDump:
                    if (wristTimer.seconds() >= wristFlipTime){
                        highBasketState = HighBasketState.HighBasket;
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        wristClaw.WristUp();
                        highBasketState = HighBasketState.WristUp;
                        wristTimer.reset();
                    }
                    break;

                case HighBasket:
                    if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        wristClaw.WristUp();
                        highBasketState = HighBasketState.WristUp;
                        wristTimer.reset();
                    }
                    break;

                case WristUp:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.ExtendShort(1);
                        highBasketState = HighBasketState.ViperRetractedShort;
                    }
                    break;

                case ViperRetractedShort:
                    if (viper.getIsViperRetractedShort()) {
                        viper.ExtendClosed(0.25);
                        highBasketState = HighBasketState.ViperClosed;
                    }

                case ViperClosed:
                    if (viper.getIsViperExtendClosed()) {
                        arm.MoveToHome();
                        highBasketState = HighBasketState.LoweringArm;
                    }
                    break;

                case LoweringArm:
                    if (arm.getIsArmHomePosition()) {
                        highBasketState = HighBasketState.Home;
                    }
                    break;
            }

            switch (specimenClipState) {
                case Home:
                    if (gamepad2.right_trigger > 0 && gamepad2.dpad_up) {
                        wristClaw.WristMid();
                        specimenClipState = SpecimenClipState.WristUp;
                        wristTimer.reset();
                    }
                    break;

                case WristUp:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        arm.MoveToSpecimen();
                        specimenClipState = SpecimenClipState.RaiseArm;
                    }
                case RaiseArm:
                    if (arm.getIsArmSpecimenPosition()) {
                        viper.ExtendSpecimenhang(1);
                        specimenClipState = SpecimenClipState.ViperExtend;
                    }
                    else if (gamepad2.right_trigger > 0 && gamepad2.dpad_down) {
                        arm.MoveToHome();
                        specimenClipState = SpecimenClipState.ArmLowerToHome;
                    }
                    break;

                case ViperExtend:
                    if (viper.getIsViperExtendSpecimenHang()) {
                        specimenClipState = SpecimenClipState.SpecimenHang;
                    }
                    else if (gamepad2.right_trigger > 0 && gamepad2.dpad_down) {
                        viper.ExtendClosed(0.25);
                        specimenClipState = SpecimenClipState.ViperExtendShort;
                    }
                    break;

                case SpecimenHang:
                    if (gamepad2.right_trigger > 0 && gamepad2.dpad_down) {
                        wristClaw.WristDown();
                        specimenClipState = SpecimenClipState.WristDown;
                        wristTimer.reset();
                    }
                    break;

                case WristDown:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.ExtendClosed(0.5);
                        specimenClipState = SpecimenClipState.ViperExtendClosed;
                    }

                case ViperExtendClosed:
                    if (viper.getIsViperExtendClosed()) {
                        arm.MoveToHome();
                        specimenClipState = SpecimenClipState.ArmLowerToHome;
                    }
                    break;

                case ArmLowerToHome:
                    if (arm.getIsArmHomePosition()) {
                        specimenClipState = SpecimenClipState.Home;
                    }



            }

            switch (submersiblePickupState) {
                case Home:
                    if (gamepad1.y) {
                        wristClaw.WristUp();
                        submersiblePickupState = SubmersiblePickupState.WristUpIn;
                        wristTimer.reset();
                    }
                    break;

                case WristUpIn:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.Extendsubmersible(0.5);
                        submersiblePickupState = SubmersiblePickupState.ExtendViperPickup;
                    }
                    break;

                case ExtendViperPickup:
                    if (viper.getIsViperExtendSub()) {
                        submersiblePickupState = SubmersiblePickupState.SubmersiblePickup;
                    }
                    break;

                case SubmersiblePickup:
                    if (gamepad1.a) {
                        wristClaw.WristUp();
                        submersiblePickupState = SubmersiblePickupState.WristUpOut;
                        wristTimer.reset();
                    }
                    break;

                case WristUpOut:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.ExtendClosed(0.5);
                        submersiblePickupState = SubmersiblePickupState.ExtendViperClosed;
                    }
                    break;

                case ExtendViperClosed:
                    if (viper.getIsViperExtendClosed()) {
                        submersiblePickupState = SubmersiblePickupState.Home;
                    }
                    break;
            }

            //Pose2d pos = odo.getPositionRR();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));


            telemetry.update();

        }

    }
    private String getPinpoint(Pose2D pos) {
        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), (pos.getHeading(AngleUnit.DEGREES)));
    }
    private String getPinpoint(Pose2d pos) {
        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));
    }
}