package frc.robot;

import static java.util.Map.entry;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

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
        public static final double kPDrive = 10;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;
        public static final double kPSteer = 6;
        public static final double kISteer = 0;
        public static final double kDSteer = 0;

        
        public static final double driveSupplyCurrentLimit = 80;

        public static final double maxDriveAcceleration = 4;
        public static final double maxAngleAcceleration = 2*Math.PI;
        public static final double maxDriveVelocity = TunerConstants.kSpeedAt12VoltsMps;
        public static final double maxAngleVelocity = 1.5*Math.PI;
        public static final double shellyDriveVelocity = maxDriveVelocity * 0.25;
        public static final double shellyAngleVelocity = maxAngleVelocity * 0.50;
        

        /*
         * FL D - 4
         * FL S - 6
         * FR D - 13
         * FR S - 15
         * BL D - 7
         * BL S - 9
         * BR D - 10
         * BR S - 12
         */
    }

    public static final class VisionConstants {
        /*
         * A note about these transforms: They appear to follow the normal cordinate
         * system (x is right when pos. and so on).
         */
        public static final Transform3d leftTransform = new Transform3d(-0.282, 0.293, 0.646, 
                                                        new Rotation3d(Units.degreesToRadians(23), Units.degreesToRadians(30), Units.degreesToRadians(-50)));
        public static final Transform3d rightTransform = new Transform3d(-0.282, -0.293, 0.646, 
                                                        new Rotation3d(Units.degreesToRadians(-23), Units.degreesToRadians(30), Units.degreesToRadians(50)));
        public static final String leftCameraName = "LeftCam";
        public static final String rightCameraName = "RightCam";

        public static final double maxAmbiguity = 0.1;
        public static final List<Integer> validTagIds = Arrays.asList(3, 4, 8, 7);
        // public static final List<Integer> validTagIds = null;

        public static final PoseStrategy mainStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;
        public static final PoseStrategy fallbackStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;
        public static final int aprilTagUpdateFrequency = 50; // in seconds, between 1 and 50.

    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 57;
        public static final int rightMotorID = 58;
        public static final int leftMotorPDPID = 4;
        public static final int rightMotorPDPID = 5;
        public static final boolean shooterMotorInvert = true;

        public static final double kP = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 8;

        public static final double shootVelo = 92.0; // Rotations per second
        public static final double shooterTolerance = 3.0;

        public static final double minShootSpeed = 50; // measured for AutoAim
        public static final double maxShootSpeed = 90; // measured for AutoAim
        public static final double spitOutSpeed = 20;

        public static final double shooterBackupSpeed = -0.1;

        public static final Map<Double, Double> rotationLookupTable = Map.ofEntries(
            entry(0.0, 0.0780),
            entry(1.41, 0.0780), // lined up
            entry(1.70, 0.0610), // 4
            entry(2.31, 0.0385), // 6
            entry(2.92, 0.0227), // 8
            entry(3.54, 0.0130), // 10
            entry(4.14, 0.0030) // 12
        );

        public static final Map<Double, Double> speedLookupTable = Map.ofEntries(
            entry(0.0, 50.0),
            entry(1.41, 50.0),
            entry(1.70, 50.0),
            entry(2.31, 50.0),
            entry(2.92, 60.0),
            entry(3.54, 70.0),
            entry(4.14, 92.0),
            entry(50.0, 92.0)
        );
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 60;
        public static final int intakeMotorPDPID = 15;
        public static final double intakeSpeed = 1;
        public static final double ejectSpeed = -1;
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
        public static final int resetGyroButton = 1;
        public static final int autoAimButton = 13;
        public static final int resetAtPointButton = 5;
        public static final int shellyButton = 2;
        public static final int leftX = 0;
        public static final int leftY = 1;
        public static final int rightX = 3;
        public static final int rightY = 2;

        public static final double deadband = 0.01;//0.06;
        public static final double leftXMax = 0.75;
        public static final double leftYMax = 0.66;
        public static final double rightXMax = 0.8;
        public static final double rightYMax = 0.8;
    }

    public static final class TransportConstants {
        public static final int transportMotorID = 56;
        public static final int transportMotorPDPID = 16;
        public static final boolean transportMotorInvert = true;
        public static final double transportSpeed = 0.60;
        public static final double transportEjectSpeed = -1;
        public static final int irSensorChannel = 4;

        public static final double transportEjectDelay = 0.3; // seconds until note leaves shooter
    }

    public static final class PositionConstants {
        public static final class StowPresets {
            public static final double elevator = 0.0;
            public static final double shooter = 0.0;
        }

        public static final class AmpPresets {
            public static final double elevator = 2.34;
            public static final double shooter = 0;
        }

        public static final class TrapPresets {
            public static final double elevator = 2.34;
            public static final double shooter = 0.088379;
        }

        public static final class TestPresets {
            public static final double elevator = 6 / (Math.PI * 1.751);
            public static final double shooter = 0.02;
        }

        public static final class ResetPresets {
            public static final double elevator = 0.25;
            public static final double shooter = 0.0;
        }

        public static final class SubwooferPresets {
            public static final double elevator = 0.0;
            public static final double shooter = 0.076;
        }
    }

    public static final class ElevatorConstants {
        public static final int elevatorMotorID = 50;
        public static final int elevatorMotorPDPID = 2;
        
        public static final int forwardLimitChannelID = 0;
        public static final int reverseLimitChannelID = 1;

        public static final double rotorToSensorRatio = 1.0;
        public static final double sensorToMechanismRatio = 25.0;

        public static final double elevatorLowerLimit = 0;
        public static final double elevatorUpperLimit = 1;

        public static final double elevatorManualUpSpeed = 0.1;
        public static final double elevatorManualDownSpeed = -0.1;

        public static final double elevatorTolerance = 0.05;

        public static final InvertedValue invertMotor = InvertedValue.Clockwise_Positive;

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

        public static final double elevatorCurrentLimit = 20;

        public static final double elevatorForwardSoftLimit = 2.33; //output shaft rotations

        public static final double gearDiameter = 1.751;
        public static final double elevatorTilt = Math.toRadians(60);

        public static final double outputShaftToInchesMultiplier = Math.PI * gearDiameter;
        public static final double inchesToOutputShaftMultiplier = 1 / outputShaftToInchesMultiplier;
    }

    public static final class PivotConstants {
        public static final int pivotMotorID = 59;
        public static final int EncoderID = 51;
        public static final double encoderOffset = -0.085693;

        public static final double rotorToSensorRatio = 125;
        public static final double sensorToMechanismRatio = 1.0;

        public static final int pivotMotorPDPID = 14;

        public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.Clockwise_Positive;

        public static final double forwardSoftLimitThreshold = 0.088379;
        public static final double reverseSoftLimitThreshold = 0;

        public static final double radiansAtZero = Math.toRadians(30);
        public static final double radiansAtMax = Math.toRadians(58);

        public static final double pivotManualUpSpeed = 0.025;
        public static final double pivotManualDownSpeed = -0.025;

        public static final double pivotCurrentLimit = 0;

        public static final double pivotTolerance = 0.004;
        
        public static final AbsoluteSensorRangeValue absoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        public static final class PivotSlot0ConfigConstants {
            public static final double kP = 150.0; //output per unit of error in position (output/rotation)
            public static final double kI = 0.0; //output per unit of integrated error in position (output/(rotation*s))
            public static final double kD = 0.0; //output per unit of error in velocity (output/rps)
            public static final double kS = 0.0; //output to overcome static friction (output)
            public static final double kV = 50.0; //output per unit of target velocity (output/rps)
            public static final double kA = 0.0; //output per unit of target acceleration (output/(rps/s))
            public static final double kG = 0.0; //feedforward Constant
        }

        public static final class PivotMotionMagicConstants {
            public static final double cruiseVelocity = 0.088379 * 50; // Target cruise velocity
            public static final double acceleration = (0.088379 * 20) * 2; // Target acceleration
            public static final double jerk = acceleration * 10; // Target Jerk
        }
    }

    public static final class AimConstants {
        public static final double minimumDistanceToNotBreakRobot = 3; // meters from speaker
        public static final double clearanceHeight = 1.9; 
        public static final double elevatorHeightFromFloorAtRest = 0.19; // ONLY Y DIRECTION

        public static final double rangeTolerance = 0.01;

        public static final double compressionAdder = 3;

        public static final double maxRange = 3; 

        public static final double speakerHeight = 2.0515; // meters
        public static final double aprilTagToHoodGoal = Units.inchesToMeters(8);

        public static final double yawMomentumGain = 0.0;
        public static final double pitchMomemtumGain = 0.0;

        public static final double dragPitchGainSlope = -0.09836;
        public static final double dragPitchGainYIntercept = 1.2;
    }

    public static final class WinchConstants {
        public static final int leftWinchMotorID = 30;
        public static final int rightWinchMotorID = 31;

        public static final int leftWinchMotorPDPID = 30;
        public static final int rightWinchMotorPDPID = 31;

        public static final boolean winchMotorInvert = true;

        public static final double sensorToMechanismRatio = 25.0;

        public static final double climbHeight = 21;
        public static final double restHeight = 4;

        public static final double winchManualUpSpeed = 0.5;
        public static final double winchManualDownSpeed = -0.5;
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
