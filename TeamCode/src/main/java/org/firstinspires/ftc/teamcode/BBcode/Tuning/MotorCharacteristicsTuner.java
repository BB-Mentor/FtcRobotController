package org.firstinspires.ftc.teamcode.BBcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Motor Characteristics Tuner", group = "Tuning")
public class MotorCharacteristicsTuner extends LinearOpMode {
    // Static parameters for FTC Dashboard
    public static String motorName = "armMotor"; // Change as needed
    public static double ticksPerRev = 537.7;
    public static double gearing = 19.2;
    public static double maxRPM = 312;
    public static boolean reverseDirection = false;

    // Test parameters
    public static double powerIncrement = 0.01;
    public static double testDurationSeconds = 2.0;
    public static double stabilizationDelaySeconds = 1.0;

    // Results (readable via Dashboard)
    public static double minPowerToMove = 0;
    public static double maxVelocity = 0;
    public static double maxAcceleration = 0;

    // Test mode constants
    private final int TEST_MIN_POWER = 0;
    private final int TEST_MAX_VELOCITY = 1;
    private final int TEST_MAX_ACCELERATION = 2;

    // Hardware
    private DcMotorEx motor;
    private FtcDashboard dashboard;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure motor parameters
        MotorConfigurationType motorType = motor.getMotorType();
        motorType.setTicksPerRev(ticksPerRev);
        motorType.setGearing(gearing);
        motorType.setMaxRPM(maxRPM);

        if (reverseDirection) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Y = Min Power Test, B = Max Velocity Test, A = Max Acceleration Test");
        telemetry.update();

        // Wait for start
        waitForStart();
        timer.reset();

        int currentTest = -1;
        boolean testRunning = false;

        while (opModeIsActive()) {
            // Handle test selection
            if (gamepad1.y && !testRunning) {
                currentTest = TEST_MIN_POWER;
                testRunning = true;
                findMinimumPowerToMove();
                testRunning = false;
            } else if (gamepad1.b && !testRunning) {
                currentTest = TEST_MAX_VELOCITY;
                testRunning = true;
                measureMaxVelocity();
                testRunning = false;
            } else if (gamepad1.a && !testRunning) {
                currentTest = TEST_MAX_ACCELERATION;
                testRunning = true;
                measureMaxAcceleration();
                testRunning = false;
            }

            // Display current state
            telemetry.addData("Status", testRunning ? "Test Running" : "Ready");
            telemetry.addData("Current Test", getTestName(currentTest));
            telemetry.addData("Results", "---------------");
            telemetry.addData("Min Power to Move", minPowerToMove);
            telemetry.addData("Max Velocity", String.format("%.2f ticks/s (%.2f rad/s)",
                    maxVelocity, ticksPerSecToRadPerSec(maxVelocity)));
            telemetry.addData("Max Acceleration", String.format("%.2f ticks/s² (%.2f rad/s²)",
                    maxAcceleration, ticksPerSecSquaredToRadPerSecSquared(maxAcceleration)));
            telemetry.addData("Current Position", String.format("%d ticks (%.2f rad)",
                    motor.getCurrentPosition(), ticksToRadians(motor.getCurrentPosition())));
            telemetry.addData("Current Velocity", String.format("%.2f ticks/s (%.2f rad/s)",
                    motor.getVelocity(), ticksPerSecToRadPerSec(motor.getVelocity())));
            telemetry.update();

            // Send data to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("current_position_ticks", motor.getCurrentPosition());
            packet.put("current_position_rad", ticksToRadians(motor.getCurrentPosition()));
            packet.put("current_velocity_ticks_per_sec", motor.getVelocity());
            packet.put("current_velocity_rad_per_sec", ticksPerSecToRadPerSec(motor.getVelocity()));
            packet.put("min_power", minPowerToMove);
            packet.put("max_velocity_ticks_per_sec", maxVelocity);
            packet.put("max_velocity_rad_per_sec", ticksPerSecToRadPerSec(maxVelocity));
            packet.put("max_acceleration_ticks_per_sec2", maxAcceleration);
            packet.put("max_acceleration_rad_per_sec2", ticksPerSecSquaredToRadPerSecSquared(maxAcceleration));
            dashboard.sendTelemetryPacket(packet);

            sleep(20); // Short delay to prevent CPU overuse
        }
    }

    private String getTestName(int test) {
        switch (test) {
            case TEST_MIN_POWER: return "Minimum Power Test";
            case TEST_MAX_VELOCITY: return "Maximum Velocity Test";
            case TEST_MAX_ACCELERATION: return "Maximum Acceleration Test";
            default: return "None";
        }
    }

    private void findMinimumPowerToMove() {
        telemetry.addData("Test", "Finding minimum power to move...");
        telemetry.update();

        // Reset motor
        motor.setPower(0);
        sleep(500);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int startPosition = motor.getCurrentPosition();

        double power = 0;
        boolean movementDetected = false;

        while (!movementDetected && power < 1.0 && opModeIsActive()) {
            power += powerIncrement;
            motor.setPower(power);

            // Wait a moment for potential movement
            sleep((long)(stabilizationDelaySeconds * 1000));

            int currentPosition = motor.getCurrentPosition();
            int movement = Math.abs(currentPosition - startPosition);

            telemetry.addData("Testing power", power);
            telemetry.addData("Movement detected", movement);
            telemetry.update();

            // Check if movement is significant (more than 5 ticks)
            if (movement > 5) {
                movementDetected = true;
                minPowerToMove = power;
            }
        }

        motor.setPower(0);

        telemetry.addData("Test Complete", "Min Power: " + minPowerToMove);
        telemetry.update();
    }

    private void measureMaxVelocity() {
        telemetry.addData("Test", "Measuring maximum velocity...");
        telemetry.update();

        // Parameters for velocity stabilization detection
        final double VELOCITY_STABILIZATION_THRESHOLD = 5.0; // ticks/sec
        final int STABLE_READINGS_REQUIRED = 10;
        final int SAMPLES_TO_AVERAGE = 5;

        // Reset motor
        motor.setPower(0);
        sleep(500);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Apply full power
        motor.setPower(1.0);

        // Initialize variables for velocity measurement
        double[] recentVelocities = new double[SAMPLES_TO_AVERAGE];
        double lastAverageVelocity = 0;
        int stableReadingsCount = 0;
        int sampleIndex = 0;
        timer.reset();

        // Safety timeout (in case velocity never stabilizes)
        double MAX_TEST_DURATION = 5.0; // seconds

        while (opModeIsActive() && timer.seconds() < MAX_TEST_DURATION) {
            // Get current velocity
            double currentVelocity = Math.abs(motor.getVelocity());

            // Update our rolling average
            recentVelocities[sampleIndex] = currentVelocity;
            sampleIndex = (sampleIndex + 1) % SAMPLES_TO_AVERAGE;

            // Calculate average velocity over the recent samples
            double sum = 0;
            for (double v : recentVelocities) {
                sum += v;
            }
            double averageVelocity = sum / SAMPLES_TO_AVERAGE;

            // Check if we have filled the average buffer
            if (timer.seconds() > (SAMPLES_TO_AVERAGE * 0.02)) {
                // Calculate change from last average
                double velocityChange = Math.abs(averageVelocity - lastAverageVelocity);

                // Check if velocity has stabilized
                if (velocityChange < VELOCITY_STABILIZATION_THRESHOLD) {
                    stableReadingsCount++;
                } else {
                    stableReadingsCount = 0;
                }

                // If we have enough stable readings, we're done
                if (stableReadingsCount >= STABLE_READINGS_REQUIRED) {
                    maxVelocity = averageVelocity;
                    break;
                }

                lastAverageVelocity = averageVelocity;
            }

            telemetry.addData("Current Velocity", String.format("%.2f ticks/s (%.2f rad/s)",
                    currentVelocity, ticksPerSecToRadPerSec(currentVelocity)));
            telemetry.addData("Average Velocity", String.format("%.2f ticks/s (%.2f rad/s)",
                    averageVelocity, ticksPerSecToRadPerSec(averageVelocity)));
            telemetry.addData("Stable Readings", stableReadingsCount + "/" + STABLE_READINGS_REQUIRED);
            telemetry.update();

            // Sleep is still necessary to:
            // 1. Prevent excessive CPU usage
            // 2. Allow time for velocity readings to update
            sleep(20);
        }

        // If we reached the timeout without stabilizing, use the last average
        if (stableReadingsCount < STABLE_READINGS_REQUIRED) {
            double sum = 0;
            for (double v : recentVelocities) {
                sum += v;
            }
            maxVelocity = sum / SAMPLES_TO_AVERAGE;
        }

        motor.setPower(0);

        telemetry.addData("Test Complete", "Max Velocity: " + maxVelocity);
        telemetry.update();
    }

    private void measureMaxAcceleration() {
        telemetry.addData("Test", "Measuring maximum acceleration...");
        telemetry.update();

        // Parameters for velocity stabilization detection
        final double VELOCITY_STABILIZATION_THRESHOLD = 5.0; // ticks/sec
        final int STABLE_READINGS_REQUIRED = 10;
        final int SAMPLES_TO_AVERAGE = 5;

        // Reset motor
        motor.setPower(0);
        sleep(500);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize measurement variables
        double[] recentVelocities = new double[SAMPLES_TO_AVERAGE];
        double lastAverageVelocity = 0;
        int stableReadingsCount = 0;
        int sampleIndex = 0;

        double previousTime = 0;
        double previousVelocity = 0;
        double maxAcc = 0;

        // Apply full power
        timer.reset();
        motor.setPower(1.0);

        // Safety timeout (in case velocity never stabilizes)
        double MAX_TEST_DURATION = 5.0; // seconds

        while (opModeIsActive() && timer.seconds() < MAX_TEST_DURATION) {
            double currentTime = timer.seconds();
            double currentVelocity = Math.abs(motor.getVelocity());

            // Calculate acceleration if we have previous measurements
            if (previousTime > 0) {
                double deltaTime = currentTime - previousTime;
                double deltaVelocity = currentVelocity - previousVelocity;

                if (deltaTime > 0) {
                    double acceleration = deltaVelocity / deltaTime;
                    if (acceleration > maxAcc) {
                        maxAcc = acceleration;
                    }
                }
            }

            // Update rolling average for velocity stabilization detection
            recentVelocities[sampleIndex] = currentVelocity;
            sampleIndex = (sampleIndex + 1) % SAMPLES_TO_AVERAGE;

            // Calculate average velocity over the recent samples
            double sum = 0;
            for (double v : recentVelocities) {
                sum += v;
            }
            double averageVelocity = sum / SAMPLES_TO_AVERAGE;

            // Check if we have filled the average buffer
            if (timer.seconds() > (SAMPLES_TO_AVERAGE * 0.02)) {
                // Calculate change from last average
                double velocityChange = Math.abs(averageVelocity - lastAverageVelocity);

                // Check if velocity has stabilized
                if (velocityChange < VELOCITY_STABILIZATION_THRESHOLD) {
                    stableReadingsCount++;
                } else {
                    stableReadingsCount = 0;
                }

                // If we have enough stable readings, we're done
                if (stableReadingsCount >= STABLE_READINGS_REQUIRED) {
                    break;
                }

                lastAverageVelocity = averageVelocity;
            }

            telemetry.addData("Current Acceleration", String.format("%.2f ticks/s² (%.2f rad/s²)",
                    (previousTime > 0) ? (currentVelocity - previousVelocity) / (currentTime - previousTime) : 0,
                    ticksPerSecSquaredToRadPerSecSquared((previousTime > 0) ? (currentVelocity - previousVelocity) / (currentTime - previousTime) : 0)));
            telemetry.addData("Current Velocity", String.format("%.2f ticks/s (%.2f rad/s)",
                    currentVelocity, ticksPerSecToRadPerSec(currentVelocity)));
            telemetry.addData("Max Acceleration (so far)", String.format("%.2f ticks/s² (%.2f rad/s²)",
                    maxAcc, ticksPerSecSquaredToRadPerSecSquared(maxAcc)));
            telemetry.addData("Stable Readings", stableReadingsCount + "/" + STABLE_READINGS_REQUIRED);
            telemetry.update();

            // Store current values for next iteration
            previousTime = currentTime;
            previousVelocity = currentVelocity;

            sleep(20);
        }

        motor.setPower(0);
        maxAcceleration = maxAcc;

        telemetry.addData("Test Complete", "Max Acceleration: " + maxAcceleration);
        telemetry.update();
    }
    private double ticksToRadians(double ticks) {
        return ticks * (2 * Math.PI / ticksPerRev);
    }

    private double ticksPerSecToRadPerSec(double ticksPerSec) {
        return ticksPerSec * (2 * Math.PI / ticksPerRev);
    }

    private double ticksPerSecSquaredToRadPerSecSquared(double ticksPerSecSquared) {
        return ticksPerSecSquared * (2 * Math.PI / ticksPerRev);
    }
}