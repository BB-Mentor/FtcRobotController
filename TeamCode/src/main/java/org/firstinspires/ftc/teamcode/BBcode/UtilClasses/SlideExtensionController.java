package org.firstinspires.ftc.teamcode.BBcode.UtilClasses;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Advanced slide controller that handles variable friction and gravity effects
 */
public class SlideExtensionController {
    private final PIDEx slidePID;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastPosition = 0;
    private double maxExtension;
    private double maxVelocity = 30.0; // inches/sec
    private double maxAcceleration = 60.0; // inches/sec²
    private double targetPosition = 0;

    public SlideExtensionController(double maxExtension) {
        // Create extended PID with anti-windup and derivative filtering
        PIDCoefficientsEx coefficients = new PIDCoefficientsEx(
                0.02,   // Kp
                0.001,  // Ki
                0.005,  // Kd
                0.5,    // maximumIntegralSum
                0.15,   // lowPassGain for derivative filtering
                10.0    // stabilityThreshold
        );
        slidePID = new PIDEx(coefficients);
        this.maxExtension = maxExtension;
    }

    /**
     * Calculate motor power for slide extension
     * @param targetExtension Target extension in inches
     * @param currentExtension Current extension in inches
     * @param armAngle Current arm angle in radians
     * @return Motor power output (-1 to 1)
     */
    public double calculate(double targetExtension, double currentExtension, double armAngle) {
        // Update target
        targetPosition = targetExtension;

        // Calculate velocity for motion profiling
        double dt = timer.seconds();
        timer.reset();
        double velocity = (currentExtension - lastPosition) / dt;
        lastPosition = currentExtension;

        // Calculate error
        double error = targetPosition - currentExtension;

        // Calculate desired velocity using motion profile
        double distance = Math.abs(error);
        double desiredVelocity = Math.signum(error) * Math.min(maxVelocity,
                Math.sqrt(2 * maxAcceleration * distance));

        // Scale down velocity when approaching target
        if (distance < 2.0) {
            desiredVelocity *= distance / 2.0;
        }

        // Get base PID output
        double pidOutput = slidePID.calculate(targetPosition, currentExtension);

        // Add gravity compensation (max at 90°)
        double gravityComp = Math.sin(armAngle) * 0.2 * (currentExtension / maxExtension);

        // Add friction compensation (max at 0°, increases with extension)
        double frictionComp = 0.15 * (currentExtension / maxExtension) *
                (1.0 - Math.abs(Math.sin(armAngle)));
        frictionComp *= Math.signum(error);

        // Velocity feedforward
        double velocityFF = 0.01 * desiredVelocity;

        // Special case for retracting at high angle
        double brakingForce = 0;
        if (error < 0 && armAngle > Math.toRadians(45) && currentExtension < 5.0) {
            brakingForce = velocity * 0.1;
        }

        // Combine all components
        double power = pidOutput + velocityFF + gravityComp + frictionComp + brakingForce;

        // Safety limits
        return Math.max(-1.0, Math.min(1.0, power));
    }
}