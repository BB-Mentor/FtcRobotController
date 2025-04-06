package org.firstinspires.ftc.teamcode.BBcode.UtilClasses;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;

import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.ArmHardwareDescription;

/**
 * Feedforward controller that computes a control output using the arm geometry.
 */
public class AdjustableLengthArmFeedforward implements FeedforwardController {
    public ArmHardwareDescription hardwareDescription;

    /**
     * Constructs a FeedforwardController using the specified hardware description.
     *
     * @param hardwareDescription the ArmHardwareDescription object containing physical parameters
     */
    public AdjustableLengthArmFeedforward(ArmHardwareDescription hardwareDescription) {
        this.hardwareDescription = hardwareDescription;
    }

//    /**
//     * Calculates the feedforward control output.
//     *
//     * @param angle               desired joint angle (radians)
//     * @param angularVelocity     desired angular velocity (rad/s)
//     * @param angularAcceleration desired angular acceleration (rad/s²)
//     * @param movableExtension    current extension of the movable portion of the slide (in inches)
//     * @return the feedforward control output (normalized to motor power between -1.0 and 1.0)
//     */
//    public double calculateFeedForward(double angle, double angularVelocity, double angularAcceleration, double movableExtension) {
//        double[] gains = hardwareDescription.getFeedforwardGains();
//        double kS = gains[0];
//        double kV = gains[1];
//
//        // Calculate each component of torque
//        double gravityTorque = hardwareDescription.calculateGravityTorque(angle, movableExtension);
//        double inertiaTorque = hardwareDescription.calculateInertiaTorque(movableExtension, angularAcceleration);
//        double dampingTorque = kV * angularVelocity;
//        double staticTorque = (angularVelocity == 0.0) ? kS : kS * Math.signum(angularVelocity);
//
//        // Total torque needed
//        double requiredTorque = inertiaTorque + dampingTorque + gravityTorque + staticTorque;
//
//        // Scale to a normalized motor power value
//        return requiredTorque / hardwareDescription.getMaxArmTorque();
//    }

    /**
     * Calculates the feedforward control output.
     *
     * @param targetAngle               desired joint angle (radians)
     * @param targetAngularVelocity     desired angular velocity (rad/s)
     * @param targetAngularAcceleration desired angular acceleration (rad/s²)
     * @return the feedforward control output (normalized to motor power between -1.0 and 1.0)
     */
    @Override
    public double calculate(double targetAngle, double targetAngularVelocity, double targetAngularAcceleration) {
        double[] gains = hardwareDescription.getFeedforwardGains();
        double kS = gains[0];
        double kV = gains[1];

        // Calculate each component of torque
        double gravityTorque = hardwareDescription.calculateGravityTorque(targetAngle, targetAngularVelocity);
        double inertiaTorque = hardwareDescription.calculateInertiaTorque(hardwareDescription.getCurrentExtension(), targetAngularAcceleration);
        double dampingTorque = kV * targetAngularVelocity;
        double staticTorque = (targetAngularVelocity == 0.0) ? kS : kS * Math.signum(targetAngularVelocity);

        // Total torque needed
        double requiredTorque = inertiaTorque + dampingTorque + gravityTorque + staticTorque;

        // Scale to a normalized motor power value
        return requiredTorque / hardwareDescription.getMaxArmTorque();
    }
}