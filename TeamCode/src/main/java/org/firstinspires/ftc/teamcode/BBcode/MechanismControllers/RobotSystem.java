package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Abstract base class for physical robotic systems and subsystems that can be arranged
 * in a hierarchical structure (e.g., chassis -> arm -> wrist -> gripper).
 */
public abstract class RobotSystem {
    // Constants
    protected static final double INCHES_TO_METERS = 0.0254;
    protected static final double GRAVITATIONAL_ACCELERATION = 9.81;

    // Parent-child relationships
    protected RobotSystem parent;
    protected List<RobotSystem> children = new ArrayList<>();

    // Relative position and orientation from parent
    protected Vector3D relativePosition = new Vector3D(0, 0, 0); // x, y, z in inches
    protected YawPitchRollAngles relativeOrientation = new YawPitchRollAngles( AngleUnit.RADIANS, 0, 0, 0, 0) ;

    /**
     * Adds a child subsystem to this system
     * @param child The child system to add
     */
    public void addChild(RobotSystem child) {
        children.add(child);
        child.parent = this;
    }

    /**
     * Sets the relative position of this system from its parent
     * @param x offset in x direction (inches)
     * @param y offset in y direction (inches)
     * @param z offset in z direction (inches)
     */
    public void setRelativePosition(double x, double y, double z) {
        relativePosition = new Vector3D(x, y, z);
    }

    /**
     * Sets the relative position of this system from its parent
     * @param position Vector3D representing position
     */
    public void setRelativePosition(Vector3D position) {
        relativePosition = position;
    }

    /**
     * Gets the relative position of this system
     * @return Vector3D with the position
     */
    public Vector3D getRelativePosition() {
        return relativePosition;
    }

    /**
     * Sets the relative orientation of this system from its parent
     * @param roll rotation around x-axis (radians)
     * @param pitch rotation around y-axis (radians)
     * @param yaw rotation around z-axis (radians)
     */
    public void setRelativeOrientation(double roll, double pitch, double yaw) {
        relativeOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, yaw, pitch, roll, 0);
    }

    /**
     * Sets the relative orientation of this system from its parent
     * @param orientation YawPitchRollAngles object
     */
    public void setRelativeOrientation(YawPitchRollAngles orientation) {
        relativeOrientation = orientation;
    }

    /**
     * Gets the relative orientation of this system
     * @return YawPitchRollAngles object
     */
    public YawPitchRollAngles getRelativeOrientation() {
        return relativeOrientation;
    }

    /**
     * Gets the mass of this system including all child systems
     * @return total mass in kg
     */
    public double getTotalMass() {
        double mass = getMass();
        for (RobotSystem child : children) {
            mass += child.getTotalMass();
        }
        return mass;
    }

    /**
     * Get center of mass (COM) of the entire system in the base frame
     * @return Vector3D representing COM in inches
     */
    public Vector3D getCenterOfMass() {
        // Initialize with this system's COM weighted by mass
        Vector3D com = getLocalCenterOfMass();
        double thisMass = getMass();
        double totalMass = thisMass;

        // Weighted sum calculation
        Vector3D weightedCom = com.scalarMultiply(thisMass);

        // Add contribution from each child
        for (RobotSystem child : children) {
            double childMass = child.getTotalMass();
            Vector3D childCOM = child.getCenterOfMass();

            // Transform child COM to parent frame
            Vector3D transformedCOM = transformToParentFrame(childCOM, child);

            // Add weighted child COM
            weightedCom = weightedCom.add(transformedCOM.scalarMultiply(childMass));
            totalMass += childMass;
        }

        // Normalize by total mass
        if (totalMass > 0) {
            weightedCom = weightedCom.scalarMultiply(1.0 / totalMass);
        }

        return weightedCom;
    }

    /**
     * Calculates moment of inertia of the system about the base frame origin
     * @return moment of inertia tensor as SimpleMatrix in kg*m²
     */
    public SimpleMatrix getMomentOfInertia() {
        // Get local inertia tensor
        SimpleMatrix totalInertia = getLocalMomentOfInertia();

        // Add contributions from all children
        for (RobotSystem child : children) {
            // Get child's inertia tensor in its local frame and transform it
            SimpleMatrix childInertia = transformChildInertia(child);

            // Add child's contribution to total inertia
            totalInertia = totalInertia.plus(childInertia);
        }

        return totalInertia;
    }
    /**
     * Transforms a child's inertia tensor to the parent frame
     */
    private SimpleMatrix transformChildInertia(RobotSystem child) {
        // Get child's local inertia tensor
        SimpleMatrix childLocalInertia = child.getLocalMomentOfInertia();

        // Get rotation matrix from child orientation
        SimpleMatrix rotMatrix = createRotationMatrix(child.getRelativeOrientation());

        // Apply rotation: R * I * R^T
        SimpleMatrix rotatedInertia = rotMatrix.mult(childLocalInertia).mult(rotMatrix.transpose());

        // Apply parallel axis theorem
        double childMass = child.getMass();
        Vector3D childPos = child.getRelativePosition();

        // Convert to meters for physics calculations
        double dx = childPos.getX() * INCHES_TO_METERS;
        double dy = childPos.getY() * INCHES_TO_METERS;
        double dz = childPos.getZ() * INCHES_TO_METERS;

        // Create identity matrix
        SimpleMatrix identity = SimpleMatrix.identity(3);

        // Create outer product matrix r⊗r
        SimpleMatrix outerProduct = new SimpleMatrix(3, 3);
        outerProduct.set(0, 0, dx*dx); outerProduct.set(0, 1, dx*dy); outerProduct.set(0, 2, dx*dz);
        outerProduct.set(1, 0, dy*dx); outerProduct.set(1, 1, dy*dy); outerProduct.set(1, 2, dy*dz);
        outerProduct.set(2, 0, dz*dx); outerProduct.set(2, 1, dz*dy); outerProduct.set(2, 2, dz*dz);

        // Calculate r²
        double r2 = dx*dx + dy*dy + dz*dz;

        // Apply parallel axis theorem: I' = I + m(r²I₃ - r⊗r)
        SimpleMatrix shiftedInertia = rotatedInertia.plus(
                identity.scale(childMass * r2).minus(outerProduct.scale(childMass))
        );

        // Recursively add contributions from the child's children
        for (RobotSystem grandchild : child.children) {
            // Transform grandchild inertia to child frame
            SimpleMatrix grandchildInertia = child.transformChildInertia(grandchild);

            // Add to child's inertia
            shiftedInertia = shiftedInertia.plus(grandchildInertia);
        }

        return shiftedInertia;
    }

    /**
     * Calculates gravity torque at the given system state
     * @return gravity torque as Vector3D in the base frame (Nm)
     */
    public Vector3D calculateGravityTorque() {
        // Initialize with local gravity torque
        Vector3D torque = getLocalGravityTorque();

        // Add torques from children
        for (RobotSystem child : children) {
            Vector3D childTorque = child.calculateGravityTorque();
            // Transform and add child torques
            torque = torque.add(childTorque);
        }

        return torque;
    }

    /**
     * Transforms a point from a child's frame to this frame
     */
    protected Vector3D transformToParentFrame(Vector3D point, RobotSystem child) {
        // Get rotation matrix
        SimpleMatrix rotMatrix = createRotationMatrix(child.getRelativeOrientation());

        // Create point vector
        double[] pointArray = {point.getX(), point.getY(), point.getZ()};
        SimpleMatrix pointVector = new SimpleMatrix(3, 1, true, pointArray);

        // Apply rotation
        SimpleMatrix rotatedPoint = rotMatrix.mult(pointVector);

        // Apply translation
        return new Vector3D(
                rotatedPoint.get(0) + child.relativePosition.getX(),
                rotatedPoint.get(1) + child.relativePosition.getY(),
                rotatedPoint.get(2) + child.relativePosition.getZ()
        );
    }

    // Abstract methods to be implemented by each specific system subclass

    /**
     * Gets the mass of just this system (excluding children)
     * @return mass in kg
     */
    protected abstract double getMass();

    /**
     * Gets center of mass in the local frame
     * @return Vector3D representing COM in inches
     */
    protected abstract Vector3D getLocalCenterOfMass();

    /**
     * Gets the local moment of inertia tensor in the local frame
     * @return 3x3 inertia tensor in kg*m²
     */
    protected abstract SimpleMatrix getLocalMomentOfInertia();

    /**
     * Gets the local gravity torque in the local frame
     * @return Vector3D with torque in Nm
     */
    protected abstract Vector3D getLocalGravityTorque();

    /**
     * Creates a rotation matrix from YawPitchRollAngles
     */
    private SimpleMatrix createRotationMatrix(YawPitchRollAngles orientation) {
        double yaw = orientation.getYaw(AngleUnit.RADIANS);
        double pitch = orientation.getPitch(AngleUnit.RADIANS);
        double roll = orientation.getRoll(AngleUnit.RADIANS);

        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cr = Math.cos(roll), sr = Math.sin(roll);

        // ZYX convention rotation matrix
        double[][] data = {
                {cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr},
                {sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr},
                {-sp, cp*sr, cp*cr}
        };

        return new SimpleMatrix(data);
    }

    /**
     * Applies parallel axis theorem using matrices
     */
    private SimpleMatrix applyParallelAxisTheorem(SimpleMatrix inertia, double mass,
                                                  double dx, double dy, double dz) {
        // Create distance squared matrix
        double r2 = dx*dx + dy*dy + dz*dz;

        // Create cross product matrix for position vector
        double[][] crossData = {
                {0, -dz, dy},
                {dz, 0, -dx},
                {-dy, dx, 0}
        };
        SimpleMatrix crossMatrix = new SimpleMatrix(crossData);

        // Apply parallel axis theorem: I' = I + m(r²I₃ - r⊗r)
        SimpleMatrix identity = SimpleMatrix.identity(3);
        SimpleMatrix outerProduct = new SimpleMatrix(3, 3);

        // Fill outer product matrix r⊗r
        outerProduct.set(0, 0, dx*dx);
        outerProduct.set(0, 1, dx*dy);
        outerProduct.set(0, 2, dx*dz);
        outerProduct.set(1, 0, dy*dx);
        outerProduct.set(1, 1, dy*dy);
        outerProduct.set(1, 2, dy*dz);
        outerProduct.set(2, 0, dz*dx);
        outerProduct.set(2, 1, dz*dy);
        outerProduct.set(2, 2, dz*dz);

        // I' = I + m(r²I₃ - r⊗r)
        return inertia.plus(identity.scale(mass * r2).minus(outerProduct.scale(mass)));
    }
}