// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.NoteFinder;
import frc.robot.subsystems.SubsystemManager;

public class RobotContainer {
  public enum RepathChoices {SHOOTER,AMP,SOURCE,NULL;}
  
  private final Joystick driver = new Joystick(InputConstants.kDriverControllerPort);
  private final JoystickButton resetGyroButton = new JoystickButton(driver, DriverConstants.resetGyroButton);
  private final JoystickButton autoAimButton = new JoystickButton(driver, DriverConstants.autoAimButton);
  private final JoystickButton resetAtPointButton = new JoystickButton(driver, DriverConstants.resetAtPointButton);
  private final JoystickButton shellyButton = new JoystickButton(driver, DriverConstants.shellyButton);

  private final Supplier<Double> driverLeftX = () -> Math.pow(MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.leftX),DriverConstants.deadband)/DriverConstants.leftXMax, 3.0);
  private final Supplier<Double> driverLeftY = () -> -Math.pow(MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.leftY), DriverConstants.deadband)/DriverConstants.leftYMax, 3.0);
  private final Supplier<Double> driverRightX = () -> Math.pow(MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.rightX), DriverConstants.deadband)/DriverConstants.rightXMax,3.0);
  // private final Supplier<Double> driverRightY = () -> driver.getRawAxis(DriverConstants.rightY)/DriverConstants.rightYMax;
  
  private final CommandXboxController xboxOperator = new CommandXboxController(InputConstants.kOperatorControllerPort);
  // private final CommandPS5Controller ps5Operator = new CommandPS5Controller(InputConstants.kOperatorControllerPort);

  SendableChooser<Command> pathChooser = new SendableChooser<>();
  private SubsystemManager subsystemManager = SubsystemManager.getInstance();
  
  private void configureDriverBindings() {
    subsystemManager.configureDriveDefaults(driverLeftY, driverLeftX, driverRightX);
    
    resetGyroButton.onTrue(subsystemManager.makeResetCommand());
    resetAtPointButton.onTrue(subsystemManager.resetAtPose2d(new Pose2d(15.1, 5.5, Rotation2d.fromDegrees(0))));
    autoAimButton.whileTrue(subsystemManager.makeAutoAimCommand(driverLeftX, driverLeftY, driverRightX));
    shellyButton.whileTrue(subsystemManager.makeShellyCommand(driverLeftX, driverLeftY, driverRightX));


    if (Utils.isSimulation()) {subsystemManager.resetAtPose2d(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));}
    subsystemManager.telemeterize();
  }

  private void configureXboxOperatorBindings() {
    xboxOperator.b().whileTrue(subsystemManager.makeShootCommand()); // shoot manually
    xboxOperator.x().whileTrue(subsystemManager.makeIntakeCommand()); // intake
    xboxOperator.a().whileTrue(subsystemManager.makeAmpScoreCommand()); // auto amp (will do everything)
    // xboxOperator.a().whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.AMP));
    xboxOperator.y().whileTrue(subsystemManager.makeTrapScoreCommand()); // auto trap (will do everything)
    // xboxOperator.y().whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.STOW));


    xboxOperator.povUp().whileTrue(subsystemManager.makeManualElevatorCommand(true));
    xboxOperator.povDown().whileTrue(subsystemManager.makeManualElevatorCommand(false));
    xboxOperator.povRight().whileTrue(subsystemManager.makeManualPivotCommand(true));
    xboxOperator.povLeft().whileTrue(subsystemManager.makeManualPivotCommand(false));

    xboxOperator.rightBumper().whileTrue(subsystemManager.makeEjectCommand());

    xboxOperator.leftBumper().whileTrue(subsystemManager.makeAutoAimCommand(() -> xboxOperator.getLeftX(), () -> xboxOperator.getLeftY(), () -> xboxOperator.getRightX()));

    // xboxOperator.back().whileTrue(subsystemManager.makeWinchCommand(true));
    // xboxOperator.start().whileTrue(subsystemManager.makeWinchCommand(false));

    xboxOperator.back().whileTrue(subsystemManager.makeManualWinchCommand(true));
    xboxOperator.start().whileTrue(subsystemManager.makeManualWinchCommand(false));


    // xboxOperator.leftTrigger(InputConstants.triggerTolerance); // left trigger as button
    // xboxOperator.leftBumper(); 
    // xboxOperator.leftStick(); // left thumbstick button
    // xboxOperator.getLeftX();
    // xboxOperator.getLeftY();
    // xboxOperator.getLeftTriggerAxis();

    // xboxOperator.rightTrigger(InputConstants.triggerTolerance); // left trigger as button
    // xboxOperator.rightStick(); // right thumbstick button
    // xboxOperator.getRightX();
    // xboxOperator.getRightY();
    // xboxOperator.getRightTriggerAxis();
  }

  // private void configurePS5OperatorBindings() {
  //   ps5Operator.square().whileTrue(subsystemManager.makeShootCommand()); // shoot manually
  //   ps5Operator.triangle().whileTrue(subsystemManager.makeIntakeCommand()); // intake
  //   ps5Operator.circle().whileTrue(subsystemManager.makeAmpScoreCommand()); // auto amp (will do everything)
  //   ps5Operator.cross().whileTrue(subsystemManager.makeTrapScoreCommand()); // auto trap (will do everything)
    
  //   ps5Operator.povUp().whileTrue(subsystemManager.makeManualElevatorCommand(true));
  //   ps5Operator.povDown().whileTrue(subsystemManager.makeManualElevatorCommand(false));
  //   ps5Operator.povRight().whileTrue(subsystemManager.makeManualPivotCommand(true));
  //   ps5Operator.povLeft().whileTrue(subsystemManager.makeManualPivotCommand(false));

  //   ps5Operator.create().whileTrue(subsystemManager.makeWinchCommand(true)); // small button above dpad
  //   ps5Operator.options().whileTrue(subsystemManager.makeWinchCommand(false)); // small button above square/triangle/etc.

  //   ps5Operator.touchpad().whileTrue(subsystemManager.makeEjectCommand()); // touchpad as button


  //   // ps5Operator.L1(); // left bumper button
  //   // ps5Operator.L2(); // left trigger as button
  //   // ps5Operator.L3(); // left thumbstick button

  //   // ps5Operator.R1(); // right bumper button
  //   // ps5Operator.R2(); // right trigger as button
  //   // ps5Operator.R3(); // right thumbstick button

  //   // ps5Operator.PS(); // Logo as button

  //   // ps5Operator.getL2Axis(); // left trigger axis
  //   // ps5Operator.getR2Axis(); // right trigger axis

  //   // ps5Operator.getLeftX();
  //   // ps5Operator.getLeftY();

  //   // ps5Operator.getRightX();
  //   // ps5Operator.getRightY();
  // }

  public RobotContainer() {
    configureDriverBindings();
    configureXboxOperatorBindings();

    pathChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", pathChooser);

    SmartDashboard.putData("Coast Elevator", subsystemManager.elevatorNeutralMode(NeutralModeValue.Coast));
    SmartDashboard.putData("Brake Elevator", subsystemManager.elevatorNeutralMode(NeutralModeValue.Brake));

    SmartDashboard.putData("Run Tests", subsystemManager.makeTestingCommand());
  }

  public Command getAutonomousCommand() {return pathChooser.getSelected();}
  public NoteFinder getNoteFinder() {return subsystemManager.getNoteFinder();}
}
