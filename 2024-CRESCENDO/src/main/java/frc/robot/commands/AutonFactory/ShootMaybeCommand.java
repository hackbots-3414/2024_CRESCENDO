// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonFactory;

import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.BaseSubsystemCommands.AimCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class ShootMaybeCommand extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Transport transport;
  private Pose2d targetPose;
  private ShooterPivot pivot;
  private Shooter shooter;
  private Command macro;
  private Logger logger = LoggerFactory.getLogger(ShootMaybeCommand.class);

  /** Creates a new ShootMaybe. */
  public ShootMaybeCommand(CommandSwerveDrivetrain drivetrain, Transport transport, ShooterPivot pivot, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.transport = transport;
    this.pivot = pivot;
    this.shooter = shooter;
    addRequirements(drivetrain); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double smallestDistance = Double.POSITIVE_INFINITY;
    Pose2d robotPose = drivetrain.getCurrentPose2d();
    List<Pose2d> availableShootingPoses = Constants.AutonFactoryConstants.shootPoses;
    
    for (Pose2d pose : availableShootingPoses) {
      double thisDistance = pose.relativeTo(robotPose).getTranslation().getNorm();
      if (thisDistance < smallestDistance) {
        smallestDistance = thisDistance;
        targetPose = pose;
      }
    }

    logger.debug("Found best shooting position at: " + targetPose.toString());

    if (!transport.getNoteInPosition()) {
      logger.warn("No note detected, not shooting");
      macro = null;
      return;
    }

    if (targetPose == null) {
      logger.error("There was an error and there was no selected starting pose, exiting");
      macro = null;
      return;
    }

    macro = new SequentialCommandGroup(
      drivetrain.makeDriveToPoseCommandV2(targetPose),
      new AimCommand(pivot, shooter, transport, drivetrain, () -> 0.0, () -> 0.0, () ->0.0, DriverStation.getAlliance()::get)
    );

    macro.schedule();
    logger.debug("Created and sceduled drive and shoot commands");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (macro == null) return true; // we never scheduled any command, so we don't have a note or anything.
    return macro.isFinished(); // wait for the macro to finish
  }
}
