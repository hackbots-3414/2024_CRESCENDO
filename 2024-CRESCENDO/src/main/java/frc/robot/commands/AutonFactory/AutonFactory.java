// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonFactory;

import java.util.ArrayList;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutonCommands.AutoIntakeCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AutonFactory extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Intake intake;
  private Transport transport;
  private ShooterPivot pivot;
  private Shooter shooter;
  private Elevator elevator;
  private ArrayList<Pose2d> poses;
  private Pose2d startingPose;
  private Logger logger = LoggerFactory.getLogger(AutonFactory.class);

  /** Creates a new AutonFactory. */
  public AutonFactory(CommandSwerveDrivetrain drivetrain, Intake intake, Transport transport, ShooterPivot pivot, Shooter shooter, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, intake, transport, pivot, shooter, elevator);
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.transport = transport;
    this.pivot = pivot;
    this.shooter = shooter;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the "encoded" path string from smartDashboard
    String pathAsString = SmartDashboard.getString("Auton Path", "").toLowerCase();
    logger.info("Auton Path received: " + pathAsString);
    startingPose = null;
    poses = new ArrayList<Pose2d>(pathAsString.length());
    Pose2d lastPose = null;
    for (int i = 0;i < pathAsString.length();i ++) {
      char c = pathAsString.charAt(i);
      
      Translation2d noteTranslation = Constants.AutonFactoryConstants.noteTranslations.getOrDefault(c, null);
      if (noteTranslation != null) {
        // we have a target pose!
        Rotation2d desiredRobotAngle = new Rotation2d();

        if (lastPose != null) {
          // recalculate desiredRobotAngle
          double dx = noteTranslation.getX() - lastPose.getX();
          double dy = noteTranslation.getY() - lastPose.getY();

          double angleToNote = Math.atan2(dy, dx);
          desiredRobotAngle = Rotation2d.fromRadians(angleToNote).rotateBy(Rotation2d.fromRadians(Math.PI)); // this will ensure that our robot drives towards the note with the intake facing it
        }

        Pose2d desiredRobotPose = new Pose2d(noteTranslation, desiredRobotAngle);
        lastPose = desiredRobotPose;
        poses.add(desiredRobotPose);
        logger.info("Added another pose onto the path:  " + desiredRobotPose.toString());
      }
      

      // check if this character is a starting pose?
      Pose2d receivedPose = Constants.AutonFactoryConstants.startingPoses.getOrDefault(c, null);
      if (receivedPose != null && startingPose == null) {
        startingPose = receivedPose;
        lastPose = startingPose;
        logger.debug("Found a starting pose: " + startingPose.toString());
      }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SequentialCommandGroup sequence = new SequentialCommandGroup();
    sequence.addCommands(new InstantCommand(() -> {
      if (startingPose != null && Constants.AutonFactoryConstants.presetStartingPose) {
        drivetrain.seedFieldRelative(startingPose);
        logger.debug("Set starting pose at " + startingPose.toString());
      }
    }));
    for (Pose2d targetPose : poses) {
      sequence.addCommands(
        new ParallelRaceGroup(
          drivetrain.makeDriveToPoseCommandV2(targetPose),
          new AutoIntakeCommand(transport, intake, elevator, pivot, shooter)
        ),
        new ShootMaybeCommand(drivetrain, transport, pivot, shooter)
      );
    }
    
    sequence.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  } 
}