package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
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

    Command currDriveCommand;

    FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.1).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    FieldCentricFacingAngle autoRequest = new SwerveRequest.FieldCentricFacingAngle().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.1).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.1).withSteerRequestType(SteerRequestType.MotionMagic);

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
        //math
    }

    @Override
    public void execute() {
        recalculate();
        elevator.setElevatorPosition(elevatorHeight);
        shooterPivot.setPivotPosition(shooterAngle);

        if (rSupplier.get() > 0.3) {
            currDriveCommand = drivetrain.applyRequest(() -> driveRequest.withVelocityX(-xSupplier.get() * SwerveConstants.maxDriveVelocity).withVelocityY(-ySupplier.get() * SwerveConstants.maxDriveVelocity).withRotationalRate(-rSupplier.get() * SwerveConstants.maxAngleVelocity));
        } else {
            currDriveCommand = drivetrain.applyRequest(() -> autoRequest.withVelocityX(-xSupplier.get() * SwerveConstants.maxDriveVelocity).withVelocityY(-ySupplier.get() * SwerveConstants.maxDriveVelocity).withTargetDirection(drivetrainRotation));
        }
    }
}