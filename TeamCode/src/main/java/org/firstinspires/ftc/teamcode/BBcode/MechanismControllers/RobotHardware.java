package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.ejml.simple.SimpleMatrix;

public class RobotHardware extends RobotSystem {
    public ArmHardwareDescription Arm;

    OpMode _Opmode;
    //constructor
    public RobotHardware(OpMode opmode){
        _Opmode = opmode;
        ArmHardwareDescription.ArmHardwareConfig _ArmConfig = new ArmHardwareDescription.ArmHardwareConfig();
        Arm = new ArmHardwareDescription(this, _Opmode.hardwareMap, _ArmConfig);
    }
    //-------------------------------------------------------------------------------

    //Creates instance of MechanismControllers
    //Viper _Viper = new Viper(_Opmode);
    //WristClaw _WristClaw = new WristClaw(_Opmode);




    @Override
    protected double getMass() {
        return 0;
    }

    @Override
    protected Vector3D getLocalCenterOfMass() {
        return null;
    }

    @Override
    protected  SimpleMatrix getLocalMomentOfInertia() {
        return new SimpleMatrix(3, 3, true, new double[]{/* inertia tensor values */});
    }

    @Override
    protected Vector3D getLocalGravityTorque() {
        return null;
    }
}
