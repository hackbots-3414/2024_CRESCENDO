// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.AutonFactory.AutonFactory;
import frc.robot.commands.BaseSubsystemCommands.ElevatorCommand.ElevatorPresets;
import frc.robot.subsystems.NoteFinder;
import frc.robot.subsystems.SubsystemManager;

public class RobotContainer {
  private static RobotContainer me = null;
  public enum JoystickChoice {PS5, XBOX;}
  public enum AutonViews {SOURCE, AMP, CENTER, DEBUG}
  
  private final Joystick driver = new Joystick(InputConstants.kDriverControllerPort);
  private final JoystickButton resetGyroButton = new JoystickButton(driver, DriverConstants.resetGyroButton);
  private final JoystickButton autoAimButton = new JoystickButton(driver, DriverConstants.autoAimButton);
  private final JoystickButton resetAtPointButton = new JoystickButton(driver, DriverConstants.resetAtPointButton);
  private final JoystickButton shellyButton = new JoystickButton(driver, DriverConstants.shellyButton);
  private final JoystickButton ampScoreButton = new JoystickButton(driver, DriverConstants.ampScoreButton);

  private final Supplier<Double> driverLeftX = () -> Math.pow(MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.leftX),DriverConstants.deadband)/DriverConstants.leftXMax, DriverConstants.expoPower) * (driver.getRawAxis(DriverConstants.leftX) >= 0.0 ? 1.0 : -1.0);
  private final Supplier<Double> driverLeftY = () -> -Math.pow(MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.leftY), DriverConstants.deadband)/DriverConstants.leftYMax, DriverConstants.expoPower) * (driver.getRawAxis(DriverConstants.leftY) >= 0.0 ? 1.0 : -1.0);
  private final Supplier<Double> driverRightX = () -> Math.pow(MathUtil.applyDeadband(driver.getRawAxis(DriverConstants.rightX), DriverConstants.deadband)/DriverConstants.rightXMax,DriverConstants.expoPower) * (driver.getRawAxis(DriverConstants.rightX) >= 0.0 ? 1.0 : -1.0);
  // private final Supplier<Double> driverRightY = () -> driver.getRawAxis(DriverConstants.rightY)/DriverConstants.rightYMax;
  
  private final CommandXboxController xboxOperator = new CommandXboxController(InputConstants.kOperatorControllerPort);
  private final CommandPS5Controller ps5Operator = new CommandPS5Controller(InputConstants.kOperatorControllerPort);

  SendableChooser<Command> pathChooser = new SendableChooser<>();
  private SubsystemManager subsystemManager = SubsystemManager.getInstance();
  
  private void configureDriverBindings() {
    subsystemManager.configureDriveDefaults(driverLeftY, driverLeftX, driverRightX);
    
    resetGyroButton.onTrue(subsystemManager.makeResetCommand());
    resetGyroButton.onFalse(subsystemManager.makeResetCommand());
    resetAtPointButton.onTrue(subsystemManager.resetAtPose2d(new Pose2d(15.1968, 5.5, Rotation2d.fromDegrees(0))));
    autoAimButton.whileTrue(subsystemManager.makeAutoAimCommand(driverLeftY, driverLeftX, driverRightX));
    shellyButton.whileTrue(subsystemManager.makeShellyCommand(driverLeftX, driverLeftY, driverRightX));
    ampScoreButton.onTrue(subsystemManager.makeAmpSequence());
    ampScoreButton.onFalse(new InstantCommand(() -> subsystemManager.stow()));
    // ampScoreButton.onFalse(subsystemManager.makeElevatorCommand(ElevatorPresets.STOW));
    
    if (Utils.isSimulation()) {subsystemManager.resetAtPose2d(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));}
    subsystemManager.telemeterize();
  }

  private void configureXboxOperatorBindings() {
    xboxOperator.x().onTrue(subsystemManager.makeAmpSetupCommand());
    xboxOperator.x().onFalse(subsystemManager.makeAmpFinishCommand());
    xboxOperator.y().whileTrue(subsystemManager.makeSubwooferShootCommand());
    xboxOperator.b().onTrue(subsystemManager.makeSpitOutCommand());
    xboxOperator.a().whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.STOW));

    xboxOperator.povUp().whileTrue(subsystemManager.makeManualElevatorCommand(true));
    xboxOperator.povDown().whileTrue(subsystemManager.makeResetElevatorCommand());
    xboxOperator.povRight().whileTrue(subsystemManager.makeManualPivotCommand(true));
    xboxOperator.povLeft().whileTrue(subsystemManager.makeManualPivotCommand(false));

    xboxOperator.back().whileTrue(subsystemManager.makeManualWinchCommand(true)); // create
    xboxOperator.start().whileTrue(subsystemManager.makeManualWinchCommand(false)); // options

    xboxOperator.leftTrigger(0.1).whileTrue(subsystemManager.makeAutoIntakeCommand()); // left trigger
    xboxOperator.rightBumper().whileTrue(subsystemManager.makeManualIntakeEjectCommand()); // right bumper
    xboxOperator.rightTrigger(0.1).whileTrue(subsystemManager.makeShootCommand()); // right trigger
    
  }

  private void configurePS5OperatorBindings() {
    ps5Operator.square().onTrue(subsystemManager.makeAmpSetupCommand()); // x
    ps5Operator.square().onFalse(subsystemManager.makeAmpFinishCommand());
    ps5Operator.triangle().whileTrue(subsystemManager.makeSubwooferShootCommand()); // y
    ps5Operator.circle().onTrue(subsystemManager.makeSpitOutCommand()); // b
    ps5Operator.cross().whileTrue(subsystemManager.makeElevatorCommand(ElevatorPresets.STOW)); // a
    
    ps5Operator.povUp().whileTrue(subsystemManager.makeManualElevatorCommand(true));
    ps5Operator.povDown().whileTrue(subsystemManager.makeResetElevatorCommand());
    ps5Operator.povRight().whileTrue(subsystemManager.makeManualPivotCommand(true));
    ps5Operator.povLeft().whileTrue(subsystemManager.makeManualPivotCommand(false));

    ps5Operator.create().whileTrue(subsystemManager.makeManualWinchCommand(false)); // back 
    ps5Operator.options().whileTrue(subsystemManager.makeManualWinchCommand(true)); // stard

    ps5Operator.L2().whileTrue(subsystemManager.makeAutoIntakeCommand()); // left trigger
    ps5Operator.R1().whileTrue(subsystemManager.makeManualIntakeEjectCommand()); // left bumper
    ps5Operator.R2().whileTrue(subsystemManager.makeShootCommand()); // right trigger
    // ps5Operator.L1().whileTrue(subsystemManager.stopShootFlywheel()); // left bumper
  }

  private RobotContainer() {
    configureDriverBindings();

    if (DriverConstants.operatorController == JoystickChoice.PS5) {
      configurePS5OperatorBindings();
    } else {
      configureXboxOperatorBindings();
    }

    // switch (DriverConstants.autonView) {
    //   case AMP:
    //     pathChooser.addOption("Wing Clear", AutoBuilder.buildAuto("WingClear"));
    //     break;

    //   case SOURCE:
        
    //     break;

    //   case CENTER:
        
    //     break;

    //   case DEBUG:
    //     pathChooser = AutoBuilder.buildAutoChooser();
    //     break;
    // }

    pathChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", pathChooser);
    SmartDashboard.putData("ManualTransportForwardCommand", subsystemManager.makeManualTransportForwardCommand());
    SmartDashboard.putData("ManualTransportBackwardsCommand", subsystemManager.makeManualTransportBackwardsCommand());

    SmartDashboard.putData("STOP Shooter", subsystemManager.stopShootFlywheel());
    SmartDashboard.putData("Manual Shooter", subsystemManager.makeManualShootCommand());
    SmartDashboard.putData("Manual Intake Eject", subsystemManager.makeManualIntakeEjectCommand());
    // SmartDashboard.putData("Amp Score", subsystemManager.makeAmpScoreCommand());
    SmartDashboard.putData("Wheel Radius Characterization", subsystemManager.makeWheelRadiusCharacterizationCommand());
  }

  public Command getAutonomousCommand() {
    return subsystemManager.makeAutonFactoryCommand();
  }

  public NoteFinder getNoteFinder() {
    return subsystemManager.getNoteFinder();
  }

  public boolean getAmpButton() {
    return ampScoreButton.getAsBoolean();
  }

  public static synchronized RobotContainer getInstance() {
    if (me == null) {
      me = new RobotContainer();
    }
    return me;
  }
}
