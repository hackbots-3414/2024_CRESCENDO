


package frc.robot.commands.AutoScoreCommands;

import java.util.function.Supplier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.AimHelper;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.AimHelper.AimStrategies;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterPivot;

public class TurnCommand extends Command {
    Logger logger = LoggerFactory.getLogger(TurnCommand.class);

    ShooterPivot shooterPivot;
    CommandSwerveDrivetrain drivetrain;

    boolean drivetrainAtGoal;
  
    Supplier<Double> xSupplier;
    Supplier<Double> ySupplier;
    Supplier<Double> rSupplier;
    Supplier<Alliance> aSupplier;

    boolean blueSide = false;

    AimOutputContainer output;

    FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    PIDController thetaController = new PIDController(0.8, 0, 0);

    public TurnCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rSupplier, Supplier<Alliance> aSupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.aSupplier = aSupplier;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(SwerveConstants.pidTurnTolerance);
        addRequirements(drivetrain);
    }

    @Override 
    public void initialize() {
        blueSide = Robot.isBlueSide();
    }

    @Override
    public void execute() {
        Pose2d robotPosition = drivetrain.getPose();
        output = AimHelper.getAimOutputs(drivetrain, blueSide, AimStrategies.LOOKUP);

        if (!DriverStation.isAutonomous()) {
            double robotRotation = robotPosition.getRotation().getRadians();
            double targetRotation = output.getDrivetrainRotation().getRadians();
            drivetrain.setControl(driveRequest.withVelocityX(xSupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withVelocityY(ySupplier.get() * SwerveConstants.maxDriveVelocity)
                                .withRotationalRate((rSupplier.get() > 0.2 || rSupplier.get() < -0.2) ? (-rSupplier.get() * SwerveConstants.maxAngleVelocity) 
                                : (thetaController.calculate(robotRotation, targetRotation) * Constants.SwerveConstants.maxAngleVelocity)));

        }

    }

    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            logger.info("Endedd via interrupt");
        }
    }
}