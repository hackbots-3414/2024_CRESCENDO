// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AprilTags;
import frc.robot.commands.ElevatorCommand.ElevatorPresets;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SubsystemManager;

public class RobotContainer {
  public enum RepathChoices {SHOOTER,AMP,SOURCE,NULL;}
  private final CommandXboxController joystick = new CommandXboxController(Constants.InputConstants.kDriverControllerPort);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
      
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry();

  private final CommandXboxController operator = new CommandXboxController(Constants.InputConstants.kOperatorControllerPort);

  SendableChooser<Command> pathChooser = new SendableChooser<>();

  public Alliance alliance;

  public SubsystemManager subsystemManager = new SubsystemManager(drivetrain);
  
  private void configureDriverBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> subsystemManager.makeDriveCommand(joystick.getLeftY(), joystick.getLeftX(), joystick.getRightX())));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));}
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RepathChoices checkForOverrides() {
    if (joystick.x().getAsBoolean()) {
      return RepathChoices.SHOOTER;
    }
    return null;
  }

  public boolean isAtSetpoint(RepathChoices commandName) {
    switch(commandName) {
      case SHOOTER:
        Pose2d target = DriverStation.getAlliance().get() == Alliance.Red ? AprilTags.RedSpeakerCenter.value.getPose2d() : AprilTags.BlueSpeakerCenter.value.getPose2d();
        return drivetrain.getPose().getTranslation().getDistance(target.getTranslation()) >= Constants.AutonConstants.speakerTolerance ? false : true;
      default:
        return false;
    }    
  }

  public Command getRepathingCommand(RepathChoices identifier) {
    if (identifier.equals(RepathChoices.SHOOTER)) {
      return alliance == Alliance.Red ? drivetrain.repathTo(AprilTags.RedSpeakerCenter) : drivetrain.repathTo(AprilTags.BlueSpeakerCenter);
    }
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

    // Left Trigger as Button
    // operator.axisGreaterThan(Constants.InputConstants.leftTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>);

    //Right Trigger as Button
    // operator.axisGreaterThan(Constants.InputConstants.rightTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>);

    // D-PAD Up
    operator.pov(0).whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.AMP));

    // D-PAD Right
    operator.pov(90).whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.STOW));

    // D-PAD Down
    operator.pov(180).whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.TRAP));

    // D-PAD Left
    // operator.pov(270).whileTrue(<ADD COMMAND>);
  }

  public RobotContainer() {
    var alliance = DriverStation.getAlliance();
    this.alliance = alliance.isPresent() ? alliance.get() : null;

    configureDriverBindings();
    configureOperatorBindings();

    SmartDashboard.putData("Auton Mode", pathChooser);

    pathChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", pathChooser);
  }

  public Command getAutonomousCommand() {
    return pathChooser.getSelected();
  }
}
