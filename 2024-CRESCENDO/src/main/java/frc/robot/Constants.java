package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
        public static final double kPDrive = 0;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;
        public static final double kPSteer = 0;
        public static final double kISteer = 0;
        public static final double kDSteer = 0;

        public static final double maxDriveVelocity = 3.92;
        public static final double maxDriveAcceleration = 3;
        public static final double maxAngleVelocity = 1.5*Math.PI;
        public static final double maxAngleAcceleration = 2*Math.PI;
    }

    public static final class AutonConstants {
        public static final double speakerTolerance = Units.feetToMeters(9.5); // METERS YOU CAN SHOOT FROM
    }

    public static final class VisionConstants {
        /*
         * A note about these transforms: They appear to follow the normal cordinate
         * system (x is right when pos. and so on).
         */
        public static final Transform3d leftTransform = new Transform3d(-0.5, 0, 0,
                new Rotation3d(0, -Math.PI / 4.0, 0)); // FIXME Give me values that are accurate please
        public static final Transform3d rightTransform = new Transform3d(0, 0, 0, new Rotation3d(0, Math.PI / 4.0, 0)); // FIXME
                                                                                                                        // Give
                                                                                                                        // me
                                                                                                                        // accurate
                                                                                                                        // values
                                                                                                                        // as
                                                                                                                        // well
                                                                                                                        // please.
        public static final String leftCameraName = "Cam2";
        public static final String rightCameraName = "Cam1";
    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 57;
        public static final int rightMotorID = 58;
        public static final int leftMotorPDPID = 4;
        public static final int rightMotorPDPID = 5;
        public static final double shootSpeed = 0.1;
        public static final boolean shooterMotorInvert = false;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 46;
        public static final int intakeMotorPDPID = 15;
        public static final double intakeSpeed = 1;
        public static final double ejectSpeed = -1;
        public static final boolean intakeMotorInvert = false;
        public static final int rightX = 3;
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int leftTriggerID = 2;
        public static final int rightTriggerID = 3;

        public static final double triggerTolerance = 0.5;
    }

    public static final class DriverConstants {
        public static final int resetGyroButton = 13;
        public static final int repathButton = 12;
        public static final int leftX = 0;
        public static final int leftY = 1;
    }

    public static final class TransportConstants {
        public static final int transportMotorID = 56;
        public static final int transportMotorPDPID = 16;
        public static final boolean transportMotorInvert = false;
        public static final double transportSpeed = 0.5;
        public static final double transportEjectSpeed = -0.5;
        public static final int irSensorChannel = 1;
    }

    public static final class PositionConstants {

        public static final class stowPresets {
            public static final double elevator = 0;
            public static final double shooter = 0;
        }

        public static final class ampPresets {
            public static final double elevator = 0;
            public static final double shooter = 0;
        }

        public static final class trapPresets {
            public static final double elevator = 0;
            public static final double shooter = 0;
        }

    }

    public static final class ElevatorConstants {
        public static final int elevatorMotorID = 50;
        public static final int elevatorFollowerMotorID = 51;
        public static final int elevatorCANCoderMotorID = 52;

        public static final int elevatorMotorPDPID = 2;
        public static final int elevatorFollowerMotorPDPID = 3;

        public static final double elevatorkS = 0;
        public static final double elevatorkG = 0;
        public static final double elevatorkV = 0;
        public static final double elevatorkA = 0;

        public static final double elevatorLowerLimit = 0;
        public static final double elevatorUpperLimit = 1;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static double maxVelocity;
        public static double maxAcceleration;
        public static double elevatorCurrentLimit;
        public static double circumference;
        public static double gearRatio;
    }

    public static final class PivotConstants {
        public static final int pivotMotorID = 0;
        public static final int EncoderID = 0;

        public static final int pivotMotorPDPID = 14;

        public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        public static final double forwardSoftLimitThreshold = 0.25;
        public static final double reverseSoftLimitThreshold = -0.25;

        public static final double motorCurrentLimit = 0;

        public static final double kP = 0; // FIX ME

        // These are fake gains; in actuality these must be determined individually for
        // each robot
        public static final double kSVolts = 0;
        public static final double kGVolts = 0;
        public static final double kVVoltSecondPerRad = 0;
        public static final double kAVoltSecondSquaredPerRad = 0;

        public static final double kMaxVelocityRadPerSecond = 0;
        public static final double kMaxAccelerationRadPerSecSquared = 0;

        // The offset of the arm from the horizontal in its neutral position,
        // measured from the horizontal in amount of total rotation.
        // Ex 0.5 is half a rotation
        public static final double kArmOffset = 0; // FIX ME
    }

    public static class AprilTagObject {
        private int ID;

        public int getID() {
            return ID;
        }

        private Pose2d position;

        public Pose2d getPose2d() {
            return position;
        }

        public AprilTagObject(int id, double x, double y, double degrees) {
            this.ID = id;
            this.position = new Pose2d(Units.inchesToMeters(x), Units.inchesToMeters(y),
                    Rotation2d.fromDegrees(degrees));
        }
    }

    public class NoteFinderConstants {
        public static final int DATAGRAM_PORT = 5800;
        public static final int BUFFER_SIZE = 512;
        public static final double CYCLE_TIME = 0.015;
    }
}
