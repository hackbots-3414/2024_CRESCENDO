package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class AimRobotCommand extends Command {
    Elevator elevator;
    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;

    double elevatorHeight;
    double shooterAngle;
    Rotation2d drivetrainRotation;
    Supplier<Double> xSupplier;
    Supplier<Double> ySupplier;
    Supplier<Double> rSupplier;

    Command currentDriveCommand;

    FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.1).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    FieldCentricFacingAngle autoRequest = new SwerveRequest.FieldCentricFacingAngle().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.1).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.1).withSteerRequestType(SteerRequestType.MotionMagic);

    double velocityParallelGain = 0.01;
    double velocityPerpendicularGain = 0.01;

    public AimRobotCommand(Elevator elevator, ShooterPivot shooterPivot, CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rSupplier) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
    }

    private void recalculate() {
        ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();
        double velocityParallel = speeds.vxMetersPerSecond;
        double velocityPerpendicular = speeds.vyMetersPerSecond;
        double yawAdd = velocityParallel * velocityParallelGain;
        double pitchAdd = velocityPerpendicular * velocityPerpendicularGain;

        drivetrainRotation = new Rotation2d(FieldConstants.speakerX, FieldConstants.speakerY).plus(Rotation2d.fromRadians(yawAdd));

        Pose2d robotPosition2d = drivetrain.getPose();
        Pose2d speakerPosition = new Pose2d(FieldConstants.speakerX, FieldConstants.speakerY, drivetrainRotation);
        Pose2d speakerRelative = speakerPosition.relativeTo(robotPosition2d);

        double v = ShooterConstants.shootSpeed;
        double g = 9.81;
        double x = speakerRelative.getTranslation().getNorm();
        double y = FieldConstants.speakerHeight - ElevatorConstants.minimumHeight;

        if (x > ElevatorConstants.minimumDistanceToNotBreakRobot) {
            elevatorHeight = 0.0;
        } else {
            elevatorHeight = ElevatorConstants.clearanceHeight;
            y += elevatorHeight * Math.sin(ElevatorConstants.elevatorTilt);
            x += elevatorHeight * Math.cos(ElevatorConstants.elevatorTilt);
        }

        double theta1 = Math.atan((v * v + Math.sqrt(v * v * v * v - g * (g * x * x + 2 * y * v * v))) / (g * x));
        double theta2 = Math.atan((v * v - Math.sqrt(v * v * v * v - g * (g * x * x + 2 * y * v * v))) / (g * x));
        double launchAngle = x > 0 ? theta1 : theta2; // radians
        if (!runTests(v, launchAngle, g, x, y)) launchAngle = launchAngle == theta1 ? theta2 : theta1;
        shooterAngle = launchAngle + pitchAdd;        
    }

    @Override
    public void execute() {
        recalculate();
        elevator.setElevatorPosition(elevatorHeight);
        shooterPivot.setPivotPosition(shooterAngle);

        if (rSupplier.get() > 0.3) {
            currentDriveCommand = drivetrain.applyRequest(() -> driveRequest.withVelocityX(-xSupplier.get() * SwerveConstants.maxDriveVelocity).withVelocityY(-ySupplier.get() * SwerveConstants.maxDriveVelocity).withRotationalRate(-rSupplier.get() * SwerveConstants.maxAngleVelocity));
        } else {
            currentDriveCommand = drivetrain.applyRequest(() -> autoRequest.withVelocityX(-xSupplier.get() * SwerveConstants.maxDriveVelocity).withVelocityY(-ySupplier.get() * SwerveConstants.maxDriveVelocity).withTargetDirection(drivetrainRotation));
        }
    }

    private boolean runTests(double velocity, double launchAngle, double gravity, double checkX, double checkY) {
        double initialVelocityX = velocity * Math.cos(launchAngle);
        double initialVelocityY = velocity * Math.sin(launchAngle);
        double timeOfFlight = (2 * initialVelocityY) / gravity;
        double horizontalDistanceTraveled = initialVelocityX * timeOfFlight;
        double verticalDistanceTraveled =initialVelocityY * timeOfFlight - 0.5 * gravity * timeOfFlight * timeOfFlight;

        return horizontalDistanceTraveled == checkX && verticalDistanceTraveled == checkY;
    }
}