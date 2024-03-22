


package frc.robot.commands.BaseSubsystemCommands;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.AimHelper;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.AimHelper.AimStrategies;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AimCommand extends Command {
    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;
    Shooter shooter;
    Transport transport;

    boolean drivetrainAtGoal;
  
    Supplier<Double> xSupplier;
    Supplier<Double> ySupplier;
    Supplier<Double> rSupplier;
    Supplier<Alliance> aSupplier;

    boolean blueSide = false;
    
    Consumer<Boolean> setDone;

    Command currentDriveCommand;
    Command shooterCommand;

    AimOutputContainer output;

    FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    PIDController thetaController = new PIDController(1, 0, 0);

    public AimCommand(ShooterPivot shooterPivot, Shooter shooter, Transport transport, CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rSupplier, Supplier<Alliance> aSupplier) {
        this.shooterPivot = shooterPivot;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.transport = transport;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.aSupplier = aSupplier;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(SwerveConstants.pidTurnTolerance);
        shooterCommand = new ShooterCommand(shooter, transport, Optional.of(this::getShooterFeedSupplier));
    }

    @Override 
    public void initialize() {
        blueSide = aSupplier.get() == Alliance.Blue;
        shooterCommand.initialize();
    }

    @Override
    public void execute() {
        Pose2d robotPosition = drivetrain.getPose();
        output = AimHelper.getAimOutputs(drivetrain, blueSide, AimStrategies.LOOKUP);
        shooterPivot.setPivotPosition(output.getPivotAngle());
        drivetrain.setInRange(output.getIsInRange());

        if (DriverStation.isTeleop()) {
            currentDriveCommand = drivetrain.applyRequest(() -> driveRequest.withVelocityX(xSupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withVelocityY(ySupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withRotationalRate((rSupplier.get() > 0.2 || rSupplier.get() < -0.2) ? (-rSupplier.get() * SwerveConstants.maxAngleVelocity) 
                                : (thetaController.calculate(robotPosition.getRotation().getRadians(), output.getDrivetrainRotation().getRadians()) * Constants.SwerveConstants.maxAngleVelocity)));

            currentDriveCommand.execute();
        } else if (DriverStation.isAutonomous()) {
            PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(output.getDrivetrainRotation()));
        }       

        shooterCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return shooterCommand.isFinished();
        // return true;
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
        currentDriveCommand.end(interrupted);
        shooterCommand.end(interrupted);
        drivetrain.setInRange(false);
    }

    public boolean getShooterFeedSupplier() {
        return thetaController.atSetpoint();
    }
}