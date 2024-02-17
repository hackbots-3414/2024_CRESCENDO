package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

import static java.util.Map.entry;

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

        public static final double maxDriveVelocity = TunerConstants.kSpeedAt12VoltsMps;
        public static final double maxDriveAcceleration = 4;
        public static final double maxAngleVelocity = 1.5*Math.PI;
        public static final double maxAngleAcceleration = 2*Math.PI;
    }

    public static final class VisionConstants {
        /*
         * A note about these transforms: They appear to follow the normal cordinate
         * system (x is right when pos. and so on).
         */
        public static final Transform3d leftTransform = new Transform3d(Units.inchesToMeters(-11.813), Units.inchesToMeters(-22.373), Units.inchesToMeters(26.25), new Rotation3d(0, Math.PI * 40/180, 0));
        public static final Transform3d rightTransform = new Transform3d(Units.inchesToMeters(11.813), Units.inchesToMeters(-22.373), Units.inchesToMeters(26.25), new Rotation3d(0, Math.PI * 40/180, 0));
        public static final String leftCameraName = "Cam2";
        public static final String rightCameraName = "Cam1";
    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 57;
        public static final int rightMotorID = 58;
        public static final int leftMotorPDPID = 4;
        public static final int rightMotorPDPID = 5;
        public static final double shootSpeed = 0.1;
        public static final boolean shooterMotorInvert = true;

        public static final double kP = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 8;

        public static final double shootVelo = 92.0; // Rotations per second

        public static final double shooterTolerance = 8.0;

        public static final Map<Double, Double> rotationLookupTable = Map.ofEntries(
            entry(0.0, 0.0),
            entry(1.0, 1.0),
            entry(2.0, 2.0),
            entry(3.0, 3.0),
            entry(4.0, 4.0),
            entry(5.0, 5.0),
            entry(6.0, 6.0),
            entry(7.0, 7.0)
        );
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 60;
        public static final int intakeMotorPDPID = 15;
        public static final double intakeSpeed = 1;
        public static final double ejectSpeed = 0;
        public static final boolean intakeMotorInvert = true;
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
        public static final int rightX = 3;
    }

    public static final class TransportConstants {
        public static final int transportMotorID = 56;
        public static final int transportMotorPDPID = 16;
        public static final boolean transportMotorInvert = true;
        public static final double transportSpeed = 0.5;
        public static final double transportEjectSpeed = -0.5;
        public static final int irSensorChannel = 2;
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
            public static final double elevator = 2.32;
            public static final double shooter = 0;
        }

        public static final class testPresets {
            public static final double elevator = 6 / (Math.PI * 1.751);
        }

    }

    public static final class ElevatorConstants {
        public static final int elevatorMotorID = 50;
        
        public static final int forwardLimitChannelID = 0;
        public static final int reverseLimitChannelID = 1;

        public static final int elevatorMotorPDPID = 2;
        public static final int elevatorFollowerMotorPDPID = 3;

        public static final double elevatorLowerLimit = 0;
        public static final double elevatorUpperLimit = 1;

        public static final double elevatorManualUpSpeed = 0.1;
        public static final double elevatorManualDownSpeed = -0.1;

        public static final class ElevatorSlot0ConfigConstants {
            public static final double kP = 15.0; //output per unit of error in position (output/rotation)
            public static final double kI = 0.0; //output per unit of integrated error in position (output/(rotation*s))
            public static final double kD = 0.0; //output per unit of error in velocity (output/rps)
            public static final double kS = 0.0; //output to overcome static friction (output)
            public static final double kV = 2.8; //output per unit of target velocity (output/rps)
            public static final double kA = 0.0; //output per unit of target acceleration (output/(rps/s))
            public static final double kG = 0.0; //Feedforward Constant
        }

        public static final class ElevatorMotionMagicConstants {
            public static final double cruiseVelocity = 4; // Target cruise velocity - 1.16
            public static final double acceleration = 8; // Target acceleration - 2.32
            public static final double jerk = 80; // Target Jerk - 23.2
        }

        public static double elevatorCurrentLimit = 20;
    }

    public static final class PivotConstants {
        public static final int pivotMotorID = 59;
        public static final int EncoderID = 51;
        public static final double encoderOffset = 0; // FIX ME

        public static final double rotorToSensorRatio = 125;
        public static final double sensorToMechanismRatio = 1.0;

        public static final int pivotMotorPDPID = 14;

        public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        public static final double forwardSoftLimitThreshold = 0.5;
        public static final double reverseSoftLimitThreshold = 0;

        public static final double pivotManualUpSpeed = 0.1;
        public static final double pivotManualDownSpeed = -0.1;

        public static final double motorCurrentLimit = 0;

        public static final class PivotSlot0ConfigConstants {
            public static final double kP = 0.0; //output per unit of error in position (output/rotation)
            public static final double kI = 0.0; //output per unit of integrated error in position (output/(rotation*s))
            public static final double kD = 0.0; //output per unit of error in velocity (output/rps)
            public static final double kS = 0.0; //output to overcome static friction (output)
            public static final double kV = 0.0; //output per unit of target velocity (output/rps)
            public static final double kA = 0.0; //output per unit of target acceleration (output/(rps/s))
            public static final double kG = 0.0; //feedforward Constant
        }

        public static final class PivotMotionMagicConstants {
            public static final double cruiseVelocity = 0.0; // Target cruise velocity
            public static final double acceleration = 0.0; // Target acceleration
            public static final double jerk = 0.0; // Target Jerk
        }
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
