// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain.AutonChoice;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(Constants.InputConstants.kDriverControllerPort); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
      
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController operator = new CommandXboxController(Constants.InputConstants.kOperatorControllerPort);

  SendableChooser<Command> pathChooser = new SendableChooser<>();

  private Alliance alliance;

  Shooter m_Shooter = new Shooter();
  Intake m_Intake = new Intake();

  private void configureDriverBindings() {
    drivetrain.setDefaultCommand(getDefaultCommand());

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private Command getDefaultCommand() {
    if (joystick.x().getAsBoolean()) {
      return alliance == Alliance.Red ? drivetrain.repathTo(AprilTags.RedSpeakerCenter, AutonConstants.speakerTolerance) : drivetrain.repathTo(AprilTags.BlueSpeakerCenter, AutonConstants.speakerTolerance);
    } else {
      return drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed).withVelocityY(-joystick.getLeftX() * MaxSpeed).withRotationalRate(-joystick.getRightX() * MaxAngularRate));
    }
  }

  private void configureOperatorBinging() {
    // operator.a().whileTrue(<ADD COMMAND>);
    operator.b().whileTrue(new ShooterCommand(m_Shooter, Constants.ShooterConstants.shootSpeed));
    operator.x().whileTrue(new IntakeCommand(m_Intake, Constants.IntakeConstants.ejectSpeed));
    // operator.y().whileTrue(<ADD COMMAND>);
    // operator.leftBumper().whileTrue(<ADD COMMAND>);
    operator.rightBumper().whileTrue(new IntakeCommand(m_Intake, Constants.IntakeConstants.intakeSpeed));
    // operator.back().whileTrue(<ADD COMMAND>);
    // operator.start().whileTrue(<ADD COMMAND>);

    // Left Trigger as Button
    // operator.axisGreaterThan(Constants.InputConstants.leftTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>);

    //Right Trigger as Button
    // operator.axisGreaterThan(Constants.InputConstants.rightTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>);

    // D-PAD Up
    // operator.pov(0).whileTrue(<ADD COMMAND>);

    // D-PAD Right
    // operator.pov(90).whileTrue(<ADD COMMAND>);

    // D-PAD Down
    // operator.pov(180).whileTrue(<ADD COMMAND>);

    // D-PAD Left
    // operator.pov(270).whileTrue(<ADD COMMAND>);
  }

  public RobotContainer() {
    var alliance = DriverStation.getAlliance();
    this.alliance = alliance.isPresent() ? alliance.get() : null;

    configureDriverBindings();
    configureOperatorBinging();

    SmartDashboard.putData("Auton Mode", pathChooser);

    pathChooser.setDefaultOption("Test Hallway", drivetrain.getAutoPath(AutonChoice.Test1));
    // pathChooser.addOption("Test Path 2", drivetrain.getAutoPath(AutonChoice.Test2));
    // pathChooser.addOption("Test Path 3", drivetrain.getAutoPath(AutonChoice.Test3));
  }

  public Command getAutonomousCommand() {
    // return pathChooser.getSelected();
    return drivetrain.getAutoPath("TestHallway");
  }
}
