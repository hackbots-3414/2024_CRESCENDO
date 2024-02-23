package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.AimHelper;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class AimRobotCommand extends Command {
    Elevator elevator;
    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;
  
    Supplier<Double> xSupplier;
    Supplier<Double> ySupplier;
    Supplier<Double> rSupplier;
    Supplier<Alliance> aSupplier;
    boolean blueSide = false;

    Command currentDriveCommand;

    FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    PIDController pidTurn = new PIDController(1, 0, 0);


    public AimRobotCommand(Elevator elevator, ShooterPivot shooterPivot, CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rSupplier, Supplier<Alliance> aSupplier) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.aSupplier = aSupplier;
    }

    @Override
    public void execute() {
        blueSide = aSupplier.get() == Alliance.Blue;
        
        // ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();
        Pose2d robotPosition = drivetrain.getPose();
        // AimOutputContainer output = AimHelper.calculateAimWithMath(robotPosition, speeds, blueSide); // WITH MATH
        AimOutputContainer output = AimHelper.calculateAimLookupTable(robotPosition, blueSide); // LOOKUP TABLE

        elevator.setElevatorPosition(output.getElevatorHeight());
        shooterPivot.setPivotPosition(output.getPivotAngle());

        Rotation2d drivetrainRotation = output.getDrivetrainRotation();

        double compensate = blueSide ? -Math.PI * 2 : 0;

        double measurement = robotPosition.getRotation().getRadians() > 0 ? robotPosition.getRotation().getRadians() + compensate : robotPosition.getRotation().getRadians();
        double setpoint = drivetrainRotation.getRadians() > 0 ? drivetrainRotation.getRadians() + compensate : drivetrainRotation.getRadians();

        SmartDashboard.putNumber("MEASUREMENT", measurement);
        SmartDashboard.putNumber("SETPOINT", setpoint);


        currentDriveCommand = drivetrain.applyRequest(() -> driveRequest.withVelocityX(xSupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withVelocityY(ySupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withRotationalRate((rSupplier.get() > 0.2 || rSupplier.get() < -0.2) ? (-rSupplier.get() * SwerveConstants.maxAngleVelocity) 
                                : (pidTurn.calculate(measurement, setpoint) * Constants.SwerveConstants.maxAngleVelocity)));

        currentDriveCommand.schedule();

        SmartDashboard.putNumber("SHOOTER ANGLE", output.getPivotAngle());
    }

    @Override
    public void end(boolean interrupted) {currentDriveCommand.cancel();}
}