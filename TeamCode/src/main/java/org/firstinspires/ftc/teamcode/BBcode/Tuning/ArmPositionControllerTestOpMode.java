package org.firstinspires.ftc.teamcode.BBcode.Tuning;

import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.ArmHardwareDescription;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.RobotHardware;
import org.firstinspires.ftc.teamcode.BBcode.UtilClasses.AdjustableLengthArmFeedforward;
import org.firstinspires.ftc.teamcode.BBcode.UtilClasses.ArmPositionController;
import org.firstinspires.ftc.teamcode.BBcode.UtilClasses.ProfiledPIDController;

@Config
@TeleOp(name = "Arm Position Controller Test", group = "Test")
public class ArmPositionControllerTestOpMode extends LinearOpMode {
    // Static parameters for FTC Dashboard
    public static double stallTorque = 2.383;
    public static double motorTicksPerRev = 537.7;
    public static double gearRatio = 1; //3.333; // 100/30
    public static double beamLength = 10.375; //12.0;
    public static double beamMass = .068; //0.5;
    public static double pivotOffset = 0.3125;
    public static double fixedSegmentStart = 0.0;
    public static int slideSegmentCount = 0;
    public static double slideSegmentLength = 0.0;
    public static double slideSegmentHeight = 0.0;
    public static double slideSegmentMass = 0.0;
    public static double slideOverlap = 0.0;
    public static double slideSideOffset = 0.0;
    public static double actuatorMass = 0; // 0.3;
    public static double actuatorOffset = 0; // 12.0;
    public static double actuatorSideOffset = 0.0;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kMs = 0.0;
    public static double kM = 0.0;
    public static double currentExtension = 0.0;

    // PID parameters
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0;
    public static double kvP = 0;
    public static double kvI = 0;
    public static double kvD = 0;
    public static double kvF = 0;
    public static double holdingKp = 0;
    public static double holdingKi = 0;
    public static double holdingKd = 0;
    public static double pitchCompensationGain = 0;
    public static double accelerationCompensationGain = 0;
    public static double holdingAngleThreshold = .175;  // radians - determines when to switch to HOLDING state
    public static double positionDeadband = .0174533;       // radians - determines deadband size
    public static double velocityDeadband;       // rad/s - determines max velocity for deadband
    public static double maxVelocity = 1; //actually max power
    public static double maxAcceleration = .5; //maximum change in power per second
    public static double angleTolerance = 0.0174533; // radians

    public enum ControlMode {
        FF("FF"),                     // Feedforward only
        PROFILED_WITH_GOAL_PID ("profiled_with_goal_PID"),
        FULL_CONTROL("posPID + velPID + FF"); // Full control

        private final String displayName;

        ControlMode(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    // Control mode
    public static ControlMode controlMode = ControlMode.FF; // Default to FF only

    // Results for graphing
    public static double armAngularVelocity = 0.0;   // rad/s
    public static double output = 0.0;       // Final motor power

    // For calculating velocity and acceleration
    private double armAngularAcceleration = 0.0;


    // Control parameters
    public static double ARM_SPEED = Math.PI / 4;  // rad/s
    private static final double TARGET_UP = Math.PI / 2;  // 90 degrees
    private static final double TARGET_DOWN = 0.0;        // 0 degrees
    private double targetAngle = 0.0;

    // Hardware and controller references
    private ArmPositionController armController;

    private FtcDashboard dashboard;
    private DcMotorEx armMotor;

    private RobotHardware robot;
    // For tracking button press transitions
    private boolean prevDpadUpButton = false;
    private boolean prevDpadDownButton = false;

    private boolean prevXButton = false;
    private boolean prevBButton = false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        robot = new RobotHardware(this);
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorConfigurationType motorType = armMotor.getMotorType();
        motorType.setTicksPerRev(537.7);
        motorType.setGearing(19.2);
        motorType.setMaxRPM(312);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Initialize the feedforward controller
        armController = new ArmPositionController(armMotor, robot.Arm, new ProfiledPIDController(kP, kI, kD, kF, new WPILibMotionProfile.Constraints(maxVelocity, maxAcceleration)));




        // Initialize the dashboard
        dashboard = FtcDashboard.getInstance();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Y = Up (90°), A = Down (0°)");
        telemetry.addData("Mode Controls", "X = FF, B = PID, Right Bumper = PIDF Combined");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get time for calculations


            // Update controller if parameters changed
            updateHardwareDescription();
            // Get current arm state
            double currentAngle = robot.Arm.getArmAngle();
            double errorAngle = targetAngle - currentAngle;
            double currentExtensionValue = robot.Arm.getMaxExtension();
            armAngularVelocity = armMotor.getVelocity(AngleUnit.RADIANS);

            // Handle mode selection with cycling
            if (gamepad1.x && !prevXButton) {
                // Decrement mode with wraparound
                controlMode = ControlMode.values()[(controlMode.ordinal() + ControlMode.values().length - 1)
                        % ControlMode.values().length];
            } else if (gamepad1.b && !prevBButton) {
                // Increment mode with wraparound
                controlMode = ControlMode.values()[(controlMode.ordinal() + 1) % ControlMode.values().length];
            }

            // Update previous button states
            prevXButton = gamepad1.x;
            prevBButton = gamepad1.b;

            // Handle target angle selection with button press detection
            boolean targetChanged = false;
            // Handle target angle selection
            // Check for dpad up button press (not hold)
            if (gamepad1.dpad_up && !prevDpadUpButton) {
                targetAngle = TARGET_UP;
                targetChanged = true;
            }
            // Check for dpad down button press (not hold)
            else if (gamepad1.dpad_down && !prevDpadDownButton) {
                targetAngle = TARGET_DOWN;
                targetChanged = true;
            }

            if (targetChanged || armController == null) {
                armController.setTargetAngle(targetAngle);
            }
            // Update previous button states
            prevDpadUpButton = gamepad1.dpad_up;
            prevDpadDownButton = gamepad1.dpad_down;


            // Calculate

            double profiledOutput = armController.calculate();



// Determine final output based on control mode
            switch (controlMode) {
                case FF:
                    output = 0;
                    break;
                case PROFILED_WITH_GOAL_PID:
                    output = 0;
                    break;
                case FULL_CONTROL:
                    output = profiledOutput;
                    break;
                default:
                    output = 0;
            }


            // Limit output to valid range
            //combinedOutput = Math.max(-1.0, Math.min(1.0, combinedOutput));

            // Apply power to motor
            armMotor.setPower(output);


            // Send data to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target_angle_deg", Math.toDegrees(targetAngle));
            packet.put("target_angle_rad", targetAngle);
            packet.put("error_angle_deg",Math.toDegrees(errorAngle));
            packet.put("error_angle_rad",errorAngle);
            packet.put("mode", getControlModeString());
            packet.put("current_angle_deg", Math.toDegrees(currentAngle));
            packet.put("current_angle_rad", currentAngle);
            packet.put("angular_velocity", armAngularVelocity);

            packet.put("combined_output", output);
            packet.put("extension", currentExtensionValue);

            dashboard.sendTelemetryPacket(packet);

            // Update telemetry
            telemetry.addData("Target Angle", String.format("%.2f° (%.2f rad)", Math.toDegrees(targetAngle), targetAngle));
            telemetry.addData("Error Angle", String.format("%.2f° (%.2f rad)", Math.toDegrees(errorAngle), errorAngle));
            telemetry.addData("Current Angle (deg)", String.format("%.2f° (%.2f rad)", Math.toDegrees(robot.Arm.getArmAngle(AngleUnit.DEGREES)), robot.Arm.getArmAngle(AngleUnit.RADIANS)));
            telemetry.addData("Current Angle (rad)", robot.Arm.getArmAngle(AngleUnit.RADIANS));
            telemetry.addData("Combined Output", output);
            telemetry.addData("Mode Controls", "X = Previous Mode, B = Next Mode (Current: " + getControlModeString() + ")");
            telemetry.addData("Controls", "Dpad-Up = Up (90°), Dpad-Down = Down (0°)");
            telemetry.addData("actual velocity", armMotor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("actual Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
    private String getControlModeString() {
        return controlMode.getDisplayName();
    }
    /**
     * Create a custom hardware description for testing purposes,
     * using the dashboard configuration parameters
     */
    private ArmHardwareDescription.ArmHardwareConfig createArmConfigFromDashboardStatics() {
        ArmHardwareDescription.ArmHardwareConfig config = new ArmHardwareDescription.ArmHardwareConfig();
        // copy the dashboard parameters to the hardware description
        config.motorStallTorque = stallTorque;
        config.ticksPerRev = motorTicksPerRev;
        config.gearRatio = gearRatio;
        config.beamLength = beamLength;
        config.beamMass = beamMass;
        config.pivotOffset = pivotOffset;
        config.fixedSegmentStart = fixedSegmentStart;
        config.slideSegmentCount = slideSegmentCount;
        config.slideSegmentLength = slideSegmentLength;
        config.slideSegmentHeight = slideSegmentHeight;
        config.slideSegmentMass = slideSegmentMass;
        config.slideOverlap = slideOverlap;
        config.slideSideOffset = slideSideOffset;
        config.actuatorMass = actuatorMass;
        config.actuatorOffset = actuatorOffset;
        config.actuatorSideOffset = actuatorSideOffset;
        config.kS = kS;
        config.kV = kV;
        config.kA = kA;

        return config;

    }
    /**
     * Update the hardware description with current dashboard parameters
     */
    private void updateHardwareDescription() {
        // Check if any parameters have changed, and if so, recreate the hardware description
        ArmHardwareDescription.ArmHardwareConfig config = createArmConfigFromDashboardStatics();
        if (!robot.Arm.isConfigEqual(config)){
            robot.Arm.updateConfig(config);
            // Update the feedforward controller with the new hardware description
            armController.getFeedforward().hardwareDescription = robot.Arm;

        }
        if (armController != null) {
            armController.getProfiledPIDController().setPID(kP, kI, kD, kF);
            armController.getProfiledPIDController().setConstraints(new WPILibMotionProfile.Constraints(maxVelocity, maxAcceleration));
        }
    }

}