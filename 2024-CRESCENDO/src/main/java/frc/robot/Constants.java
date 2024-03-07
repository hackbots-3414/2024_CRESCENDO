package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer.JoystickChoice;
import frc.robot.generated.TunerConstants;
import frc.robot.util.FieldConstants;

public class Constants {
    public static final class SwerveConstants {
        public static final double kPDrive = 10;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;
        public static final double kPSteer = 6;
        public static final double kISteer = 0;
        public static final double kDSteer = 0;

        public static final double driveSupplyCurrentLimit = 80;

        public static final double maxDriveAcceleration = 4;
        public static final double maxAngleAcceleration = 2 * Math.PI;
        public static final double maxDriveVelocity = TunerConstants.kSpeedAt12VoltsMps;
        public static final double maxAngleVelocity = 1.5 * Math.PI;
        public static final double shellyDriveVelocity = maxDriveVelocity * 0.25;
        public static final double shellyAngleVelocity = maxAngleVelocity * 0.50;

        // Need to change below Constants.
        // If using TunerX estimator,which uses default values for standard deviations
        // for odometry are 0.1 for x,y and
        // theta.
        // Default for Vision estimator are 0.9, 0.9, 0.9

        public static final double xyStdDevCoefficient = 0.01;
        public static final double thetaStdDevCoefficient = 1.0;

        public static final double pidTurnTolerance = 0.1; // radians
    }

    public static final class VisionConstants {

        public static final boolean USE_VISION = true; // Vision enabled or not

        /*
         * A note about these transforms: They appear to follow the normal cordinate
         * system (x is right when pos. and so on).
         */
        public static final Transform3d leftTransform = new Transform3d(-0.281, 0.291, 0.636,
                new Rotation3d(Units.degreesToRadians(-2.5), Units.degreesToRadians(-30), Units.degreesToRadians(-10)));
        public static final Transform3d rightTransform = new Transform3d(-0.281, -0.291, 0.636,
                new Rotation3d(Units.degreesToRadians(2.5), Units.degreesToRadians(-30), Units.degreesToRadians(10)));

        public static final String leftCameraName = "LeftCam";
        public static final String rightCameraName = "RightCam";

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10;

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), 
                        // if these numbers are less than one, multiplying will do bad things
                        1, // x
                        1, // y
                        1 * Math.PI // theta
                );
                
        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
         * radians.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), 
                        // if these numbers are less than one, multiplying will do bad things
                        .1, // x
                        .1, // y
                        .1);

        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose
        // to the opposite alliance
        public static final Pose2d FLIPPING_POSE = new Pose2d(
                new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth), new Rotation2d(Math.PI));
    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 57;
        public static final int rightMotorID = 58;
        public static final boolean shooterMotorInvert = true;

        public static final double kP = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 8;

        public static final double shootVelo = 92.0; // Rotations per second
        public static final double shooterTolerance = 3.0;

        public static final double minShootSpeed = 50; // measured for AutoAim
        public static final double maxShootSpeed = 90; // measured for AutoAim
        public static final double spitOutSpeed = 20.0;

        public static final double shooterBackupSpeed = -0.1;

        public static final Map<Double, Double> rotationLookupTable = Map.ofEntries(
                entry(0.0, 0.18),
                entry(1.41, 0.18), // lined up
                entry(1.56, 0.12), // 4
                entry(1.9, 0.105), // 6
                entry(2.2, 0.092), // 8
                entry(2.55, 0.081), // 10
                entry(4.14, 0.0030) // 12
        );

        // public static final Map<Double, Double> speedLookupTable = Map.ofEntries(
        //         entry(0.0, 50.0),
        //         entry(1.41, 50.0),
        //         entry(1.70, 50.0),
        //         entry(2.31, 50.0),
        //         entry(2.92, 60.0),
        //         entry(3.54, 70.0),
        //         entry(4.14, 92.0),
        //         entry(50.0, 92.0));
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 60;
        public static final double intakeSpeed = 0.8;
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
        public static final int resetAtPointButton = 12;
        public static final int shellyButton = 2;
        public static final int leftX = 0;
        public static final int leftY = 1;
        public static final int rightX = 3;
        public static final int rightY = 2;

        public static final double deadband = 0.01;// 0.06;
        public static final double leftXMax = 0.75;
        public static final double leftYMax = 0.66;
        public static final double rightXMax = 0.8;
        public static final double rightYMax = 0.8;

        public static final double expoPower = 2.0;

        public static final JoystickChoice operatorController = JoystickChoice.XBOX;
    }

    public static final class TransportConstants {
        public static final int transportMotorID = 56;
        public static final boolean transportMotorInvert = true;
        public static final double transportSpeed = 0.83;
        public static final double ejectSpeed = -0.6;
        public static final double transportEjectSpeed = -0.5;
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
            public static final double shooter = 0.135;
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
            public static final double shooter = 0.135;
        }
    }

    public static final class ElevatorConstants {
        public static final int elevatorMotorID = 50;

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
            public static final double kP = 15.0; // output per unit of error in position (output/rotation)
            public static final double kI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
            public static final double kD = 0.0; // output per unit of error in velocity (output/rps)
            public static final double kS = 0.0; // output to overcome static friction (output)
            public static final double kV = 2.8; // output per unit of target velocity (output/rps)
            public static final double kA = 0.0; // output per unit of target acceleration (output/(rps/s))
            public static final double kG = 0.0; // Feedforward Constant
        }

        public static final class ElevatorMotionMagicConstants {
            public static final double cruiseVelocity = 4; // Target cruise velocity - 1.16
            public static final double acceleration = 8; // Target acceleration - 2.32
            public static final double jerk = 80; // Target Jerk - 23.2
        }

        public static final double elevatorForwardSoftLimit = 2.33; // output shaft rotations

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

        public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.Clockwise_Positive;

        public static final double forwardSoftLimitThreshold = 0.135;
        public static final double reverseSoftLimitThreshold = 0;

        public static final double radiansAtZero = Math.toRadians(30);
        public static final double radiansAtMax = Math.toRadians(58);

        public static final double pivotManualUpSpeed = 0.3;
        public static final double pivotManualDownSpeed = -0.3;

        public static final double pivotTolerance = 0.004;

        public static final AbsoluteSensorRangeValue absoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        public static final class PivotSlot0ConfigConstants {
            public static final double kP = 150.0; // output per unit of error in position (output/rotation)
            public static final double kI = 0.0; // output per unit of integrated error in position
                                                 // (output/(rotation*s))
            public static final double kD = 0.0; // output per unit of error in velocity (output/rps)
            public static final double kS = 0.0; // output to overcome static friction (output)
            public static final double kV = 50.0; // output per unit of target velocity (output/rps)
            public static final double kA = 0.0; // output per unit of target acceleration (output/(rps/s))
            public static final double kG = 0.0; // feedforward Constant
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

        public static final double maxRange = 2.5;

        public static final double aprilTagToHoodGoal = Units.inchesToMeters(8);

        public static final double yawMomentumGain = 0.0;
        public static final double pitchMomemtumGain = 0.0;

        public static final double dragPitchGainSlope = -0.09836;
        public static final double dragPitchGainYIntercept = 1.2;

        public static final double speakerY = Units.inchesToMeters(218.42);
        public static final double speakerXBlue = Units.inchesToMeters(0);
        public static final double speakerXRed = Units.inchesToMeters(652.73);
        public static final double speakerHeight = Units.inchesToMeters(80.515); // (82.90 + 78.13) / 2

        public static final Pose2d blueSpeakerPos = new Pose2d(speakerXBlue, speakerY, new Rotation2d(0));
        public static final Pose2d redSpeakerPos = new Pose2d(speakerXRed, speakerY, new Rotation2d(Math.PI));

        public static final double speakerHeightMinusElevatorRaise = speakerHeight - elevatorHeightFromFloorAtRest;
        public static final double gravity = 9.81;
        public static final double velocity = ShooterConstants.maxShootSpeed * Units.inchesToMeters(1.5) * Math.PI + AimConstants.compressionAdder;
    }

    public static final class WinchConstants {
        public static final int leftWinchMotorID = 30;
        public static final int rightWinchMotorID = 31;

        public static final boolean winchMotorInvert = true;

        public static final double sensorToMechanismRatio = 25.0;

        public static final double winchManualUpSpeed = 0.5;
        public static final double winchManualDownSpeed = -0.5;
    }

    public class NoteFinderConstants {
        public static final int DATAGRAM_PORT = 5800;
        public static final int BUFFER_SIZE = 512;
        public static final double CYCLE_TIME = 0.015;
    }

    public static final class DebugConstants {
        public static final boolean debugMode = true; // setting this to true will increase your network table traffic.
    }

    private class PDPConstants {
        public static final int pdp = 1;
        public static final int winchLeft = 3;
        public static final int winchRight = 13;
        public static final int shooterLeft = 0;
        public static final int shooterRight = 1;
        public static final int elevator = 17;
        public static final int pivot = 5;
        public static final int intake = 18;
        public static final int transport = 2;

        public static final int frontLeftDrive = 6;
        public static final int frontLeftSteer = 4;
        public static final int backLeftDrive = 9;
        public static final int backLeftSteer = 7;
        public static final int frontRightDrive = 15;
        public static final int frontRightSteer = 13;
        public static final int backRightDrive = 12;
        public static final int backRightSteer = 10;
    }

    public class CurrentLimits {
        public static final double intakeStatorLimit = 35;
        public static final double intakeSupplyLimit = 0;
        
        public static final double transportStatorLimit = 20;
        public static final double transportSupplyLimit = 0;

        public static final double winchStatorLimit = 0;
        public static final double winchSupplyLimit = 0;

        public static final double shooterStatorLimit = 0;
        public static final double shooterSupplyLimit = 0;

        public static final double shooterPivotStatorLimit = 0;
        public static final double shooterPivotSupplyLimit = 0;

        public static final double elevatorStatorLimit = 0;
        public static final double elevatorSupplyLimit = 20;

        public static final double drivetrainStatorLimit = 0;
        public static final double drivetrainSupplyLimit = 80;
    }
}
