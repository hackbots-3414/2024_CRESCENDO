package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {
    public static final class VisionConstants {
        public static final Transform3d leftTransform  = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)); // FIXME Give me values that are accurate please
        public static final Transform3d rightTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)); // FIXME Give me accurate values as well please.
        public static final String leftCameraName = "FIXME"; // FIXME update to the correct name for the LEFT CAMEA on the robot.
        public static final String rightCameraName = "FIXME"; // FIXME update to the correct name
    }
}