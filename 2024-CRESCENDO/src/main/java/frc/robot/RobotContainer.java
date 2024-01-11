// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants;

public class RobotContainer {
  private double MaxSpeed = 4.5; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final CommandXboxController driver = new CommandXboxController(
      Constants.InputConstants.kDriverControllerPort);
  private final XboxController operator = new XboxController(
      Constants.InputConstants.kOperatorControllerPort);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final JoystickButton rightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton leftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

  private final JoystickButton bButton = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton aButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton xButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton yButton = new JoystickButton(operator, XboxController.Button.kY.value);

  private final JoystickButton startButton = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton backButton = new JoystickButton(operator, XboxController.Button.kBack.value);

  private final JoystickButton leftStickButton = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
  private final JoystickButton rightStickButton = new JoystickButton(operator, XboxController.Button.kRightStick.value);

  private final POVButton dPadUp = new POVButton(operator, 0);
  private final POVButton dPadLeft = new POVButton(operator, 90);
  private final POVButton dPadDown = new POVButton(operator, 180);
  private final POVButton dPadRight = new POVButton(operator, 270);

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Subsystems */
  private final Shooter m_Shooter = new Shooter();
  private final Intake m_Intake = new Intake();

  private void configureDriverBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                         // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  private void configureOperatorBindings() {
    rightBumper.whileTrue(new ShooterCommand(m_Shooter));
    leftBumper.whileTrue(new IntakeCommand(m_Intake, Constants.IntakeConstants.intakeSpeed));
    bButton.whileTrue(new IntakeCommand(m_Intake, Constants.IntakeConstants.ejectSpeed));
  }

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
