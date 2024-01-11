package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {
    public static final class IntakeConstants {
        public static final int motorID = 1;
    }

    public static final class VisionConstants {
        public static final List<Pose3d> aprilTags = Arrays.asList(new Pose3d(), new Pose3d(), new Pose3d()); // LEAVE the first one with the default values. It does not matter because therre is no april tag with id 0 // Please make these actually correct for the AprilTags
        public static final Transform3d leftTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)); // FIXME Give me values that are accurate please
        public static final Transform3d rightTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)); // FIXME Give me accurate values as well please.
    }
}