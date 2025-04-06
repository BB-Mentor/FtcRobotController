package org.firstinspires.ftc.teamcode.BBcode.UtilClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.ArmHardwareDescription;

/**
 * Advanced arm controller that delegates to ProfiledPIDController and switches to position holding
 */
public class ArmPositionController {
    private final DcMotorEx motor;
    private final ArmHardwareDescription armHardware;
    private final AdjustableLengthArmFeedforward feedforward;
    private final ProfiledPIDController profiledPIDController;

    // State tracking
    private double targetAngle;   // radians
    private boolean isPositionHoldMode = false;
    private int motorPositionTarget = 0;

    // Thresholds
    private double positionThreshold = Math.toRadians(2.0); // 2 degrees
    private double velocityThreshold = Math.toRadians(5.0); // 5 degrees/sec
    private double setpointChangeThreshold = Math.toRadians(1.0); // 1 degree

    // Timer for monitoring elapsed time when using ProfiledPIDController
    private ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor for the arm position controller
     */
    public ArmPositionController(DcMotorEx motor, ArmHardwareDescription armHardware,
                                 ProfiledPIDController profiledPIDController) {
        this.motor = motor;
        this.armHardware = armHardware;
        this.profiledPIDController = profiledPIDController;

        // Initialize feedforward controller
        feedforward = new AdjustableLengthArmFeedforward(armHardware);

        // Configure motor
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set a new target position for the arm
     */
    public void setTargetAngle(double targetAngle) {
        double currentAngle = armHardware.getArmAngle();
        // Check if target has changed significantly
        if (Math.abs(targetAngle - this.targetAngle) > setpointChangeThreshold) {
            // Switch back to profile mode if in position hold
            if (isPositionHoldMode) {
                isPositionHoldMode = false;
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Reset the ProfiledPIDController with current position

                profiledPIDController.reset(currentAngle); //Extremely important. Otherwise, it doesn't work when lowering
            }

            timer.reset();
        }
        profiledPIDController.reset(currentAngle); //Extremely important. Otherwise, it doesn't work when lowering
        this.targetAngle = targetAngle;
    }

    /**
     * Update the controller and apply motor power
     */
    public double calculate() {
        double currentAngle = armHardware.getArmAngle();
        double currentAngularVelocity = motor.getVelocity(AngleUnit.RADIANS);

        if (!isPositionHoldMode) {
            // Use the ProfiledPIDController to generate output
            double pidOutput = profiledPIDController.calculate(currentAngle, this.targetAngle);

            // Calculate feedforward
            double gravityFF = feedforward.calculate(
                    currentAngle,
                    currentAngularVelocity,
                    0 // We'll let the profiled controller handle acceleration
            );

            // Combine outputs
            double output = pidOutput + gravityFF;
            output = Math.max(-1.0, Math.min(1.0, output));

            // Apply power
            motor.setPower(output);

            // Check if we can switch to position hold
            double positionError = Math.abs(this.targetAngle - currentAngle);
            double velocityError = Math.abs(currentAngularVelocity);

            if (positionError < positionThreshold && velocityError < velocityThreshold) {
                // Switch to position hold
                isPositionHoldMode = true;
                motorPositionTarget = angleToTicks(this.targetAngle);

                motor.setTargetPosition(motorPositionTarget);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1.0);  // Full power for holding

                return 1.0;
            }

            return output;
        } else {
            // In position hold mode, just return current power
            return motor.getPower();
        }
    }

    /**
     * Convert angle in radians to motor ticks
     */
    private int angleToTicks(double angleRadians) {
        return (int)((angleRadians * armHardware.getConfig().ticksPerRev *
                armHardware.getConfig().gearRatio) / (2 * Math.PI));
    }

    /**
     * Check if controller is in position hold mode
     */
    public boolean isInPositionHoldMode() {
        return isPositionHoldMode;
    }

    /**
     * Get current target position in radians
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Set the thresholds for mode switching
     */
    public void setThresholds(double positionThreshold, double velocityThreshold,
                              double setpointChangeThreshold) {
        this.positionThreshold = positionThreshold;
        this.velocityThreshold = velocityThreshold;
        this.setpointChangeThreshold = setpointChangeThreshold;
    }

    public AdjustableLengthArmFeedforward getFeedforward() {
        return feedforward;
    }
    public ProfiledPIDController getProfiledPIDController() {
        return profiledPIDController;
    }
}