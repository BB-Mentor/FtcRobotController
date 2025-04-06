package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * Physical description of the arm system - dimensions, mass, inertia, etc.
 */
public class ArmHardwareDescription extends RobotSystem {
    // Configuration object
    private ArmHardwareConfig config;

    // Constants
    protected static final double INCHES_TO_METERS = 0.0254;
    protected static final double GRAVITATIONAL_ACCELERATION = 9.81;

    // Motor and servo hardware
    private final DcMotorEx armMotor;
    //private final Servo wristServo;
    //private final DcMotorEx slideMotor;

    /**
     * Configuration class for arm hardware parameters
     */
    public static class ArmHardwareConfig {
        // Hardware specifications
        public String armMotorName = "armMotor";
        public String wristServoName = "wrist_servo";
        public String slideMotorName = "slide_motor";
        public double ticksPerRev = 537.7;
        public double inchesPerTick = 0.01;
        public double maxExtensionInches = 20.0;

        // Arm config parameters
        public double motorStallTorque = 2.38301595; // (24.3 kg*cm from Gobilda 5202 312 RPM - (338 oz-in)
        public double gearRatio = 100.0 / 30.0; // 3.33:1

        // Beam parameters (in inches and kg)
        public double beamLength = 12.0;
        public double beamMass = 0.6;
        public double pivotOffset = 1.0;

        // Fixed segment parameters
        public double fixedSegmentStart = 2.0;

        // Slide parameters
        public int slideSegmentCount = 4;
        public double slideSegmentLength = 10.0;
        public double slideSegmentHeight = 0.25;
        public double slideSegmentMass = 0.2;
        public double slideOverlap = 2.0;
        public double slideSideOffset = 1.0;

        // Actuator parameters
        public double actuatorMass = 0.3;
        public double actuatorOffset = 0.0;
        public double actuatorSideOffset = 0.0;

        // Feedforward gains
        public double kS = 0.0;
        public double kV = 0.0;
        public double kA = 0.0;


        /**
         * Default constructor with standard values
         */
        public ArmHardwareConfig() {}


    }
    /**
     * Compares a configuration object with the current configuration to check for equality
     *
     * @param newConfig The configuration to compare against the current one
     * @return true if configurations are equal, false otherwise
     */
    public boolean isConfigEqual(ArmHardwareConfig newConfig) {
        if (newConfig == null) {
            return false;
        }

        // Compare all configuration properties
        return config.armMotorName.equals(newConfig.armMotorName) &&
                config.wristServoName.equals(newConfig.wristServoName) &&
                config.slideMotorName.equals(newConfig.slideMotorName) &&
                Double.compare(config.ticksPerRev, newConfig.ticksPerRev) == 0 &&
                Double.compare(config.inchesPerTick, newConfig.inchesPerTick) == 0 &&
                Double.compare(config.maxExtensionInches, newConfig.maxExtensionInches) == 0 &&
                Double.compare(config.motorStallTorque, newConfig.motorStallTorque) == 0 &&
                Double.compare(config.gearRatio, newConfig.gearRatio) == 0 &&
                Double.compare(config.beamLength, newConfig.beamLength) == 0 &&
                Double.compare(config.beamMass, newConfig.beamMass) == 0 &&
                Double.compare(config.pivotOffset, newConfig.pivotOffset) == 0 &&
                Double.compare(config.fixedSegmentStart, newConfig.fixedSegmentStart) == 0 &&
                config.slideSegmentCount == newConfig.slideSegmentCount &&
                Double.compare(config.slideSegmentLength, newConfig.slideSegmentLength) == 0 &&
                Double.compare(config.slideSegmentHeight, newConfig.slideSegmentHeight) == 0 &&
                Double.compare(config.slideSegmentMass, newConfig.slideSegmentMass) == 0 &&
                Double.compare(config.slideOverlap, newConfig.slideOverlap) == 0 &&
                Double.compare(config.slideSideOffset, newConfig.slideSideOffset) == 0 &&
                Double.compare(config.actuatorMass, newConfig.actuatorMass) == 0 &&
                Double.compare(config.actuatorOffset, newConfig.actuatorOffset) == 0 &&
                Double.compare(config.actuatorSideOffset, newConfig.actuatorSideOffset) == 0 &&
                Double.compare(config.kS, newConfig.kS) == 0 &&
                Double.compare(config.kV, newConfig.kV) == 0 &&
                Double.compare(config.kA, newConfig.kA) == 0;
    }
    /**
     * Constructor for hardware description with custom configuration
     */
    public ArmHardwareDescription(RobotSystem parent, HardwareMap hardwareMap, ArmHardwareConfig config) {
        this.config = config;

        // Initialize hardware
        armMotor = hardwareMap.get(DcMotorEx.class, config.armMotorName);
        //wristServo = hardwareMap.get(Servo.class, config.wristServoName);
        //slideMotor = hardwareMap.get(DcMotorEx.class, config.slideMotorName);

    }

    /**
     * Updates the hardware configuration
     */
    public void updateConfig(ArmHardwareConfig newConfig) {
        this.config = newConfig;
    }

    /**
     * Gets the current hardware configuration
     */
    public ArmHardwareConfig getConfig() {
        return config;
    }


    /**
     * Get the maximum extension available
     */
    public double getMaxExtension() {
        return config.maxExtensionInches;
    }

    /**
     * Get arm angle
     */
    public double getArmAngle(AngleUnit unit) {
        // Get motor position and convert to angle
        int position = armMotor.getCurrentPosition();
        double angleInRadians = (2 * Math.PI * position) / (config.ticksPerRev * config.gearRatio);
        return unit == AngleUnit.DEGREES ? Math.toDegrees(angleInRadians) : angleInRadians;
    }
    public double getArmAngle() {
        return getArmAngle(AngleUnit.RADIANS);
    }

    public double getCurrentExtension() {
        //return slideMotor.getCurrentPosition() * config.inchesPerTick;
        return 0;
    }
    /**
     * Gets the beam's center-of-mass measured in inches from the pivot
     */
    public double getBeamCOM() {
        return (config.beamLength / 2.0) - config.pivotOffset;
    }

    /**
     * Gets the total slide assembly length in inches
     */
    public double getTotalSlideLength() {
        return config.slideSegmentCount * config.slideSegmentLength - (config.slideSegmentCount - 1) * config.slideOverlap;
    }

    /**
     * Gets the center-of-mass of the fixed slide segment
     */
    public double getFixedSegmentCOM() {
        return config.fixedSegmentStart + config.slideSegmentLength / 2.0;
    }

    /**
     * Gets the maximum extension of the movable portion
     */
    public double getMovableMaxExtension() {
        return getTotalSlideLength() - config.slideSegmentLength;
    }

    /**
     * Gets the fully extended slide COM
     */
    public double getFullyExtendedSlideCOM() {
        return getFixedSegmentCOM() + 0.5 * getMovableMaxExtension();
    }

    /**
     * Gets the slide assembly center-of-mass given the movable extension
     */
    public double getSlideCOM(double movableExtension) {
        double maxMovableExtension = getMovableMaxExtension();
        double t = (maxMovableExtension > 0) ? movableExtension / maxMovableExtension : 0.0;
        return (1 - t) * getFixedSegmentCOM() + t * getFullyExtendedSlideCOM();
    }

    /**
     * Gets the position of the actuator's center of mass from the pivot
     */
    public double getActuatorCOMPosition(double movableExtension) {
        double slideEndPosition = config.fixedSegmentStart + config.slideSegmentLength +
                Math.min(movableExtension, getMovableMaxExtension());
        return slideEndPosition + config.actuatorOffset;
    }

    /**
     * Gets the total mass of the slide assembly
     */
    public double getTotalSlideMass() {
        return config.slideSegmentCount * config.slideSegmentMass;
    }

    /**
     * Gets the total mass of the entire assembly
     */
    public double getTotalMass() {
        return config.beamMass + getTotalSlideMass() + config.actuatorMass;
    }

    @Override
    protected double getMass() {
        return config.beamMass + getTotalSlideMass() + config.actuatorMass;
    }

    @Override
    protected Vector3D getLocalCenterOfMass() {
        return null;
    }

    @Override
    protected SimpleMatrix getLocalMomentOfInertia() {
        return new SimpleMatrix(3, 3, true, new double[]{/* inertia tensor values */});
    }

    @Override
    protected Vector3D getLocalGravityTorque() {
        return null;
    }

    /**
     * Gets the maximum torque available at the arm axle
     */
    public double getMaxArmTorque() {
        return config.motorStallTorque * config.gearRatio;
    }

    /**
     * Calculates moment of inertia of the arm system about the pivot
     */
    public double getMomentOfInertia(double movableExtension) {
        double clampedExtension = Math.max(0, Math.min(movableExtension, getMovableMaxExtension()));

        // Convert dimensions to meters
        double beamLengthMeters = config.beamLength * INCHES_TO_METERS;
        double slideSideOffsetMeters = config.slideSideOffset * INCHES_TO_METERS;
        double slideCOMMeters = getSlideCOM(clampedExtension) * INCHES_TO_METERS;
        double actuatorDistMeters = getActuatorCOMPosition(clampedExtension) * INCHES_TO_METERS;
        double actuatorSideOffsetMeters = config.actuatorSideOffset * INCHES_TO_METERS;

        // Calculate beam inertia
        double beamInertia = (config.beamMass * Math.pow(beamLengthMeters, 2)) / 3.0;

        // Calculate slide assembly inertia
        double slideInertia = getTotalSlideMass() * (Math.pow(slideCOMMeters, 2) +
                Math.pow(slideSideOffsetMeters, 2));

        // Calculate actuator inertia
        double actuatorInertia = config.actuatorMass * (Math.pow(actuatorDistMeters, 2) +
                Math.pow(actuatorSideOffsetMeters, 2));

        // Total moment of inertia is the sum of the components
        return beamInertia + slideInertia + actuatorInertia;
    }

    /**
     * Calculates gravity torque at the given angle and extension
     */
    public double calculateGravityTorque(double angle, double movableExtension) {
        double clampedExtension = Math.max(0, Math.min(movableExtension, getMovableMaxExtension()));

        // Calculate individual torque components
        double beamTorque = calculateBeamGravityTorque(angle);
        double slideTorque = calculateSlideGravityTorque(angle, clampedExtension);
        double actuatorTorque = calculateActuatorGravityTorque(angle, clampedExtension);

        // Total gravity torque
        return beamTorque + slideTorque + actuatorTorque;
    }

    private double calculateBeamGravityTorque(double angle) {
        double beamCOMInches = getBeamCOM();
        double beamCOMMeters = beamCOMInches * INCHES_TO_METERS;
        return config.beamMass * GRAVITATIONAL_ACCELERATION * beamCOMMeters * Math.cos(angle);
    }

    private double calculateSlideGravityTorque(double angle, double clampedExtension) {
        if (config.slideSegmentCount <= 0) {
            return 0.0;
        }

        double slideTorque = 0.0;
        double slideSegmentHeightMeters = config.slideSideOffset * INCHES_TO_METERS;
        double maxPossibleExtension = getMovableMaxExtension();
        double extensionRatio = clampedExtension / maxPossibleExtension;

        // Handle fixed segment separately
        slideTorque += calculateFixedSegmentTorque(angle, slideSegmentHeightMeters);

        // Handle movable segments based on extension
        if (config.slideSegmentCount > 1) {
            double retractedSegments = (config.slideSegmentCount - 1) * (1 - extensionRatio);
            double extendedSegments = (config.slideSegmentCount - 1) * extensionRatio;

            // Add torque from retracted segments
            if (retractedSegments > 0) {
                slideTorque += calculateRetractedSegmentsTorque(angle, retractedSegments, slideSegmentHeightMeters);
            }

            // Add torque from extended segments
            if (extendedSegments > 0) {
                slideTorque += calculateExtendedSegmentsTorque(angle, extendedSegments, clampedExtension, slideSegmentHeightMeters);
            }
        }

        return slideTorque;
    }

    private double calculateFixedSegmentTorque(double angle, double slideSegmentHeightMeters) {
        double fixedSegmentCOMInches = getFixedSegmentCOM();
        double fixedSegmentCOMMeters = fixedSegmentCOMInches * INCHES_TO_METERS;

        double effectiveRadius = calculateEffectiveRadius(fixedSegmentCOMMeters, slideSegmentHeightMeters);
        double effectiveAngle = calculateEffectiveAngle(angle, slideSegmentHeightMeters, fixedSegmentCOMMeters);

        return config.slideSegmentMass * GRAVITATIONAL_ACCELERATION * effectiveRadius * Math.cos(effectiveAngle);
    }

    private double calculateRetractedSegmentsTorque(double angle, double retractedSegments, double slideSegmentHeightMeters) {
        double fixedSegmentCOMInches = getFixedSegmentCOM();
        double fixedSegmentCOMMeters = fixedSegmentCOMInches * INCHES_TO_METERS;

        double retractedSideOffset = slideSegmentHeightMeters * (retractedSegments + 1) / 2;
        double effectiveRadius = calculateEffectiveRadius(fixedSegmentCOMMeters, retractedSideOffset);
        double effectiveAngle = calculateEffectiveAngle(angle, retractedSideOffset, fixedSegmentCOMMeters);

        return retractedSegments * config.slideSegmentMass * GRAVITATIONAL_ACCELERATION * effectiveRadius * Math.cos(effectiveAngle);
    }

    private double calculateExtendedSegmentsTorque(double angle, double extendedSegments, double clampedExtension, double slideSegmentHeightMeters) {
        double fixedSegmentCOMInches = getFixedSegmentCOM();
        double fixedSegmentCOMMeters = fixedSegmentCOMInches * INCHES_TO_METERS;

        double avgExtendedSegmentPos = fixedSegmentCOMMeters + (clampedExtension * INCHES_TO_METERS / 2);
        double effectiveRadius = calculateEffectiveRadius(avgExtendedSegmentPos, slideSegmentHeightMeters);
        double effectiveAngle = calculateEffectiveAngle(angle, slideSegmentHeightMeters, avgExtendedSegmentPos);

        return extendedSegments * config.slideSegmentMass * GRAVITATIONAL_ACCELERATION * effectiveRadius * Math.cos(effectiveAngle);
    }

    private double calculateActuatorGravityTorque(double angle, double clampedExtension) {
        double actuatorCOMInches = getActuatorCOMPosition(clampedExtension);
        double actuatorCOMMeters = actuatorCOMInches * INCHES_TO_METERS;
        double actuatorSideOffsetMeters = config.actuatorSideOffset * INCHES_TO_METERS;

        double effectiveRadius = calculateEffectiveRadius(actuatorCOMMeters, actuatorSideOffsetMeters);
        double effectiveAngle = calculateEffectiveAngle(angle, actuatorSideOffsetMeters, actuatorCOMMeters);

        return config.actuatorMass * GRAVITATIONAL_ACCELERATION * effectiveRadius * Math.cos(effectiveAngle);
    }

    private double calculateEffectiveRadius(double distanceMeters, double offsetMeters) {
        return Math.sqrt(Math.pow(distanceMeters, 2) + Math.pow(offsetMeters, 2));
    }

    private double calculateEffectiveAngle(double angle, double offsetMeters, double distanceMeters) {
        return angle - Math.atan2(offsetMeters, distanceMeters);
    }

    /**
     * Calculates the inertia torque based on the moment of inertia and angular acceleration
     */
    public double calculateInertiaTorque(double movableExtension, double angularAcceleration) {
        double momentOfInertia = getMomentOfInertia(movableExtension);
        return momentOfInertia * angularAcceleration;
    }

    /**
     * Get feedforward gains
     */
    public double[] getFeedforwardGains() {
        return new double[]{config.kS, config.kV, config.kA};
    }
}