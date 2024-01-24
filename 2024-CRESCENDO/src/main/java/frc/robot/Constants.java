package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public enum AprilTags {
        BlueWallSideSource(new AprilTagObject(1, 593.68, 9.68, 120)),
        BlueSpeakerSideSource(new AprilTagObject(2, 637.21, 34.79, 120)),
        RedSpeakerOffset(new AprilTagObject(3, 652.73, 196.17, 180)),
        RedSpeakerCenter(new AprilTagObject(4, 652.73, 218.42, 180)),
        RedAmp(new AprilTagObject(5, 578.77, 323.00, 270)),
        BlueAmp(new AprilTagObject(6, 72.5, 323.00, 270)),
        BlueSpeakerCenter(new AprilTagObject(7, -1.5, 218.42, 0)),
        BlueSpeakerOffset(new AprilTagObject(8, -1.5, 196.17, 0)),
        RedSpeakerSideSource(new AprilTagObject(9, 14.02, 34.79, 60)),
        RedWallSideSource(new AprilTagObject(10, 57.54, 9.68, 60)),
        RedStageFacingSource(new AprilTagObject(11, 468.69, 146.19, 300)),
        RedStageFacingSpeaker(new AprilTagObject(12, 468.69, 177.10, 60)),
        RedStageFacingBlue(new AprilTagObject(13, 441.74, 161.62, 180)),
        BlueStageFacingRed(new AprilTagObject(14, 209.48, 161.62, 0)),
        BlueStageFacingSpeaker(new AprilTagObject(15, 182.73, 177.10, 120)),
        BlueStageFacingSource(new AprilTagObject(16, 182.73, 146.19, 240));

        public final AprilTagObject value;

        AprilTags(AprilTagObject value) {
            this.value = value;
        }
    }

    public static final class SwerveConstants {
        public static final double kPDrive = 2;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;
        public static final double kPSteer = 3.8;
        public static final double kISteer = 0;
        public static final double kDSteer = 0;

        public static final double shootingRange = Units.feetToMeters(9.5);
    }

    public static final class AutonConstants {
        public static final double speakerTolerance = 9.5; // METERS YOU CAN SHOOT FROM
    }

    public static final class VisionConstants {
        /* A note about these transforms: They appear to follow the normal cordinate system (x is right when pos. and so on). */
        public static final Transform3d leftTransform  = new Transform3d(-0.5, 0, 0, new Rotation3d(0, -Math.PI / 4.0, 0)); // FIXME Give me values that are accurate please
        public static final Transform3d rightTransform = new Transform3d(0, 0, 0, new Rotation3d(0, Math.PI / 4.0, 0)); // FIXME Give me accurate values as well please.
        public static final String leftCameraName = "Cam2";
        public static final String rightCameraName = "Cam1";
    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 57;
        public static final int rightMotorID = 58;
        public static final double shootSpeed = 0.1;
        public static final boolean shooterMotorInvert = false;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 60;
        public static final double intakeSpeed = 0.5;
        public static final double ejectSpeed = -0.5;
        public static final boolean intakeMotorInvert = false;
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int leftTriggerID = 2;
        public static final int rightTriggerID = 3;

        public static final double triggerTolerance = 0.5;
    }

    public static class AprilTagObject {
        private int ID;
        public int getID() {return ID;}

        private Pose2d position;
        public Pose2d getPose2d() {return position;}

        public AprilTagObject(int id, double x, double y, double degrees) {
            this.ID = id;
            this.position = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        }
    }
}
