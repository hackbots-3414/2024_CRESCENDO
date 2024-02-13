package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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

    double velocityParallelGain = 1.01;
    double velocityPerpendicularGain = 1.01;

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
        double velocityToward = speeds.vyMetersPerSecond;

        Pose2d robotPosition2d = drivetrain.getPose();

        Pose3d shooterPosition3d = new Pose3d(robotPosition2d.getX(), robotPosition2d.getY(), elevatorHeight, new Rotation3d(0, shooterPivot.getCancoderPos(), robotPosition2d.getRotation().getDegrees()));

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
}