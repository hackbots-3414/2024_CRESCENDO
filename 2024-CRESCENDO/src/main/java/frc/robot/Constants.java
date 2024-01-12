package frc.robot;

public class Constants {
    public static final class ShooterConstants {
        public static final int leftMotorID = 40;
        public static final int rightMotorID = 41;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 0;
        public static final double intakeSpeed = 0.5;
        public static final double ejectSpeed = -0.5;
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int leftTriggerID = 2;
        public static final int rightTriggerID = 3;

        public static final double triggerTolerance = 0.5;
    }
}
