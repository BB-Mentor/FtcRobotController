package org.firstinspires.ftc.teamcode.BBcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.ArmHardwareDescription;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.RobotHardware;
import org.firstinspires.ftc.teamcode.BBcode.UtilClasses.AdjustableLengthArmFeedforward;

@Config
@TeleOp(name = "Arm Feedforward Test", group = "Test")
public class ArmFeedforwardTestOpMode extends LinearOpMode {
    // Static parameters for FTC Dashboard
    public static double stallTorque = 2.383;
    public static double motorTicksPerRev = 537.7;
    public static double gearRatio = 1; //3.333; // 100/30
    public static double beamLength = 0; //12.0;
    public static double beamMass = 0; //0.5;
    public static double pivotOffset = 0.0;
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
    public static double kS = 0.1;
    public static double kV = 0.05;
    public static double kA = 0.01;
    public static double currentExtension = 0.0;

    // Results for graphing
    //public static double armAngle = 0.0;             // radians
    public static double armAngularVelocity = 0.0;   // rad/s
    public static double gravityFF = 0.0;    // motor power (-1.0 to 1.0)

    // For calculating velocity and acceleration
    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    private double lastAngle = 0;
    private double lastVelocity = 0;
    private double armAngularAcceleration = 0.0;

    // Control parameters
    public static double ARM_SPEED = Math.PI / 4;  // rad/s
    private static final double TARGET_UP = Math.PI / 2;  // 90 degrees
    private static final double TARGET_DOWN = 0.0;        // 0 degrees
    private double targetAngle = 0.0;

    // Hardware and controller references
    //private ArmHardwareDescription hardwareDescription;
    private AdjustableLengthArmFeedforward feedforward;
    private FtcDashboard dashboard;
    private DcMotorEx armMotor;

    private RobotHardware robot;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        robot = new RobotHardware(this);
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        MotorConfigurationType motorType = armMotor.getMotorType();
        motorType.setTicksPerRev(537.7);
        motorType.setGearing(19.2);
        motorType.setMaxRPM(312);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Create custom hardware description
        //hardwareDescription = createCustomHardwareDescription();

        // Initialize the feedforward controller
        feedforward = new AdjustableLengthArmFeedforward(robot.Arm);

        // Initialize the dashboard
        dashboard = FtcDashboard.getInstance();

        // Reset timer
        timer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // Reset timer after start
        timer.reset();
        lastTime = timer.seconds();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get time for calculations
            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;

            // Update controller if parameters changed
            updateHardwareDescription();

            // Handle button inputs
            if (gamepad1.y) {
                targetAngle = TARGET_UP;
            } else if (gamepad1.a) {
                targetAngle = TARGET_DOWN;
            } else {
                targetAngle = robot.Arm.getArmAngle();
            }

            // Calculate feedforward
            gravityFF = feedforward.calculate(
                    robot.Arm.getArmAngle(), 0, 0);

            // Apply power to motor
            armMotor.setPower(gravityFF);


            // Send data to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target_angle_deg", Math.toDegrees(targetAngle));
            packet.put("FF_current_angle_deg", Math.toDegrees(robot.Arm.getArmAngle(AngleUnit.DEGREES)));
            packet.put("FF_angular_velocity", armAngularVelocity);
            packet.put("feedforward_output", gravityFF);
            packet.put("extension", currentExtension);
            dashboard.sendTelemetryPacket(packet);

            // Update telemetry
            telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle));
            telemetry.addData("FF Current Angle (deg)", Math.toDegrees(robot.Arm.getArmAngle(AngleUnit.DEGREES)));
            telemetry.addData("Feedforward Output", gravityFF);
            telemetry.addData("Controls", "Y = Up (90°), A = Down (0°)");
            telemetry.addData("actual velocity", armMotor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("actual Position", armMotor.getCurrentPosition());
            telemetry.update();

            // Save values for next iteration
            lastTime = currentTime;
            //lastAngle = armAngle;
            lastVelocity = armAngularVelocity;

        }
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
            feedforward = new AdjustableLengthArmFeedforward(robot.Arm);
        }
    }
}