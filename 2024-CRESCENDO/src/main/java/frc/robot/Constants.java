package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
    public static final class VisionConstants {
        public static final Transform3d leftTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)); // FIXME Give
                                                                                                           // me values
                                                                                                           // that are
                                                                                                           // accurate
                                                                                                           // please
        public static final Transform3d rightTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)); // FIXME
                                                                                                            // Give me
                                                                                                            // accurate
                                                                                                            // values as
                                                                                                            // well
                                                                                                            // please.
        public static final String leftCameraName = "FIXME"; // FIXME update to the correct name for the LEFT CAMEA on
                                                             // the robot.
        public static final String rightCameraName = "FIXME"; // FIXME update to the correct name
    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 40;
        public static final int rightMotorID = 41;
        public static final double shootSpeed = 0.1;
        public static final boolean shooterMotorInvert = false;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 30;
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
}