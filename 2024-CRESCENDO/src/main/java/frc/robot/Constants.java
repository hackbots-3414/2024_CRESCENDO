package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    }

    public static final class XboxControllerButtons {

        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kBack = 7;
        public static final int kStart = 8;
        public static final int kLeftStick = 9;
        public static final int kRightStick = 10;

    }
}