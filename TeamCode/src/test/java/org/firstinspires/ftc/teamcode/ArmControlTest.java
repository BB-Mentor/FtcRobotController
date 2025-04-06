package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.BBcode.LimeLightTesting;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class ArmControlTest {

    @Before
    public void setUp() {

    }

    @Test
    public void testDriveToAprilTag() {
        // Set up the botPose and botYaw
//        limeLightTesting.botPose = new Pose3D(new Position(DistanceUnit.INCH, 40, 40, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 30, 0, 0, 0));
//        limeLightTesting.botYaw = 30;
//
//        // Call the method
//        limeLightTesting.DriveToAprilTag();
//
//        // Check the drive, turn, and strafe values
//        assertTrue("Drive value is out of range", limeLightTesting.drive >= -1 && limeLightTesting.drive <= 1);
//        assertTrue("Turn value is out of range", limeLightTesting.turn >= -1 && limeLightTesting.turn <= 1);
//        assertTrue("Strafe value is out of range", limeLightTesting.strafe >= -1 && limeLightTesting.strafe <= 1);

        // Additional checks can be added based on expected behavior
    }

    @Test
    public void testAlwaysFails() {
        fail("This test always fails with this message.");
    }
}
