// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonFactory;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
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
    String pathAsString = SmartDashboard.getString("Auton Path", "");
    startingPose = null;
    poses = new ArrayList<Pose2d>(pathAsString.length());
    for (int i = 0;i < pathAsString.length();i ++) {
      char c = pathAsString.toLowerCase().charAt(i);
      Pose2d receivedPose = Constants.AutonFactoryConstants.notePoses.getOrDefault(c, null);
      if (receivedPose != null) {
        poses.add(receivedPose);
      }
      receivedPose = Constants.AutonFactoryConstants.startingPoses.getOrDefault(c, null);
      if (receivedPose != null && startingPose == null) {
        startingPose = receivedPose;
      }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SequentialCommandGroup sequence = new SequentialCommandGroup();
    sequence.addCommands(new InstantCommand(() -> {
      if (startingPose != null && Constants.AutonFactoryConstants.presetStartingPose) drivetrain.seedFieldRelative(startingPose);
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