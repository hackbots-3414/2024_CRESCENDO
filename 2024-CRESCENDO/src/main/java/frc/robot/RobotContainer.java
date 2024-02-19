// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.NoteFinder;
import frc.robot.subsystems.SubsystemManager;

public class RobotContainer {
  public enum RepathChoices {SHOOTER,AMP,SOURCE,NULL;}
  
  private final Joystick driver = new Joystick(Constants.InputConstants.kDriverControllerPort);
  private final JoystickButton resetGyroButton = new JoystickButton(driver, Constants.DriverConstants.resetGyroButton);
  private final JoystickButton autoAimButton = new JoystickButton(driver, DriverConstants.autoAimButton);

  private final Supplier<Double> driverLeftX = () -> driver.getRawAxis(Constants.DriverConstants.leftX);
  private final Supplier<Double> driverLeftY = () -> driver.getRawAxis(Constants.DriverConstants.leftY);
  private final Supplier<Double> driverRightX = () -> driver.getRawAxis(Constants.DriverConstants.rightX);
  private final Supplier<Double> driverRightY = () -> driver.getRawAxis(Constants.DriverConstants.rightY);
  
  private final CommandXboxController operator = new CommandXboxController(Constants.InputConstants.kOperatorControllerPort);

  SendableChooser<Command> pathChooser = new SendableChooser<>();
  private SubsystemManager subsystemManager = SubsystemManager.getInstance();
  
  private void configureDriverBindings() {
    subsystemManager.configureDriveDefaults(driverLeftY, driverLeftX, driverRightX);
    
    resetGyroButton.onTrue(subsystemManager.makeResetCommand());
    autoAimButton.whileTrue(subsystemManager.makeAutoAimCommand(driverLeftX, driverLeftY, driverRightX));


    if (Utils.isSimulation()) {subsystemManager.resetAtPose2d(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));}
    subsystemManager.telemeterize();
  }

  private void configureOperatorBindings() {
    operator.b().whileTrue(subsystemManager.makeShootCommand()); // shoot manually
    operator.x().whileTrue(subsystemManager.makeIntakeCommand()); // intake
    operator.a().whileTrue(subsystemManager.makeAmpScoreCommand()); // auto amp (will do everything)
    operator.y().whileTrue(subsystemManager.makeTrapScoreCommand()); // auto trap (will do everything)

    operator.rightBumper().whileTrue(subsystemManager.makeEjectCommand());

    operator.back().whileTrue(subsystemManager.makeWinchCommand(true));
    operator.start().whileTrue(subsystemManager.makeWinchCommand(false));

    // operator.axisGreaterThan(Constants.InputConstants.leftTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>); // Left Trigger as Button
    // operator.axisGreaterThan(Constants.InputConstants.rightTriggerID, Constants.InputConstants.triggerTolerance).whileTrue(<ADD COMMAND>); //Right Trigger as Button

    operator.pov(0).whileTrue(subsystemManager.makeManualElevatorCommand(true));// D-PAD Up
    operator.pov(180).whileTrue(subsystemManager.makeManualElevatorCommand(false));// D-PAD Down
    operator.pov(90).whileTrue(subsystemManager.makeManualPivotCommand(true));// D-PAD Up
    operator.pov(270).whileTrue(subsystemManager.makeManualPivotCommand(false));// D-PAD Down
  }

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();

    pathChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", pathChooser);

    SmartDashboard.putData("Coast Elevator", subsystemManager.elevatorNeutralMode(NeutralModeValue.Coast));
    SmartDashboard.putData("Brake Elevator", subsystemManager.elevatorNeutralMode(NeutralModeValue.Brake));

    SmartDashboard.putData("Run Tests", subsystemManager.makeTestingCommand());
  }

  public Command getAutonomousCommand() {return pathChooser.getSelected();}
  public NoteFinder getNoteFinder() {return subsystemManager.getNoteFinder();}
}
