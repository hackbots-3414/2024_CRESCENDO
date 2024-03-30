// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Supplier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteFinder;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.NoteFinder.Gamepiece;

public class AimAtNote extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private Intake intake;
  private NoteFinder notefinder;
  private Transport transport;

  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> rSupplier;

  private static Logger LOG = LoggerFactory.getLogger(AimAtNote.class);

  private FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.2).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private PIDController thetaController = new PIDController(0.8, 0, 0);

  /** Creates a new AimAtNote. */
  public AimAtNote(CommandSwerveDrivetrain drivetrain, NoteFinder notefinder, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rSupplier, Intake intake, Transport transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.notefinder = notefinder;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rSupplier = rSupplier;
    this.intake = intake;
    this.transport = transport;

    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(SwerveConstants.pidTurnTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LOG.debug("Started");
  }

  private Rotation2d gamepieceAngleToRobotRelative(Gamepiece gamepiece, Rotation2d currentRobotRotation){
    Rotation2d gamepieceRotation = Rotation2d.fromDegrees(gamepiece.getAngle()).rotateBy(Rotation2d.fromDegrees(180));
    return currentRobotRotation.rotateBy(gamepieceRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!notefinder.isNoteDetected() || transport.getNoteOnBoard()) {return;}
    Pose2d robotPosition = drivetrain.getPose();

    Gamepiece gamepiece = notefinder.getGamepieces()[0];

    Rotation2d robotHeading = robotPosition.getRotation();

    Rotation2d targetRotation = gamepieceAngleToRobotRelative(gamepiece, robotHeading);

    drivetrain.setControl(driveRequest.withVelocityX(xSupplier.get() * SwerveConstants.maxDriveVelocity)
    .withVelocityY(ySupplier.get() * SwerveConstants.maxDriveVelocity)
    .withRotationalRate((rSupplier.get() > 0.2 || rSupplier.get() < -0.2) ? (-rSupplier.get() * SwerveConstants.maxAngleVelocity) 
    : (-thetaController.calculate(robotPosition.getRotation().getRadians(), targetRotation.getRadians()) * Constants.SwerveConstants.maxAngleVelocity)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIntakeIR() || !notefinder.isNoteDetected() || transport.getNoteOnBoard();
  }
}
