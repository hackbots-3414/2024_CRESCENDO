// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AprilTags;
import frc.robot.commands.ElevatorCommand.ElevatorPresets;
import frc.robot.subsystems.SubsystemManager;

public class RobotContainer {
  public enum RepathChoices {SHOOTER,AMP,SOURCE,NULL;}
  
  private final CommandXboxController joystick = new CommandXboxController(Constants.InputConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(Constants.InputConstants.kOperatorControllerPort);

  SendableChooser<Command> pathChooser = new SendableChooser<>();

  public SubsystemManager subsystemManager = new SubsystemManager();
  
  private void configureDriverBindings() {
    subsystemManager.configureDriveDefaults(() -> joystick.getLeftY(), () -> joystick.getLeftX(), () -> joystick.getRightX());

    joystick.a().whileTrue(subsystemManager.makeBrakeCommand());
    joystick.b().whileTrue(subsystemManager.makePointCommand(joystick.getLeftX(), joystick.getLeftY()));
    joystick.leftBumper().onTrue(subsystemManager.makeResetCommand());

    if (Utils.isSimulation()) {subsystemManager.resetAtPose2d(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));}
    subsystemManager.telemeterize();
  }

  public RepathChoices checkForOverrides() {
    if (joystick.x().getAsBoolean()) return RepathChoices.SHOOTER;
    return null;
  }
  public boolean isAtSetpoint(RepathChoices commandName) {
    if (commandName == RepathChoices.SHOOTER) return subsystemManager.setpointCalculate(DriverStation.getAlliance().get() == Alliance.Red ? AprilTags.RedSpeakerCenter.value.getPose2d() : AprilTags.BlueSpeakerCenter.value.getPose2d(), Constants.AutonConstants.speakerTolerance);
    return false;
  }
  public Command getRepathingCommand(RepathChoices identifier) {
    if (identifier.equals(RepathChoices.SHOOTER)) return DriverStation.getAlliance().get() == Alliance.Red ? subsystemManager.makeRepathCommand(AprilTags.RedSpeakerCenter) : subsystemManager.makeRepathCommand(AprilTags.BlueSpeakerCenter);
    return null;
  }

  private void configureOperatorBindings() {
    // operator.a().whileTrue(<ADD COMMAND>);
    operator.b().whileTrue(subsystemManager.makeShootCommand(Constants.ShooterConstants.shootSpeed));
    operator.x().whileTrue(subsystemManager.makeIntakeCommand());
    // operator.y().whileTrue(<ADD COMMAND>);
    // operator.leftBumper().whileTrue(<ADD COMMAND>);
    operator.rightBumper().whileTrue(subsystemManager.makeEjectCommand());
    // operator.back().whileTrue(<ADD COMMAND>);
    // operator.start().whileTrue(<ADD COMMAND>);

    // operator.axisGreaterThan(Constants.InputConstants.leftTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>); // Left Trigger as Button
    // operator.axisGreaterThan(Constants.InputConstants.rightTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>); //Right Trigger as Button

    operator.pov(0).whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.AMP));// D-PAD Up
    operator.pov(90).whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.STOW));// D-PAD Right
    operator.pov(180).whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.TRAP));// D-PAD Down
    // operator.pov(270).whileTrue(<ADD COMMAND>); // D-PAD Left
  }

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();

    pathChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", pathChooser);
  }

  public Command getAutonomousCommand() {return pathChooser.getSelected();}
  public NoteFinder getNoteFinder() {return m_NoteFinder;}
}
