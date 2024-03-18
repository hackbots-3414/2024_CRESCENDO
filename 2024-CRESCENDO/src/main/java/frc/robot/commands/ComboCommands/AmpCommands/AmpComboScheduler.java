// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ComboCommands.StowElevatorCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AmpComboScheduler extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Elevator elevator;
  private ShooterPivot pivot;
  private Shooter shooter;
  private Transport transport;
  /** Creates a new AmpComboSceduler. */
  public AmpComboScheduler(CommandSwerveDrivetrain drivetrain, Elevator elevator, ShooterPivot pivot, Shooter shooter, Transport transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.pivot = pivot;
    this.shooter = shooter;
    this.transport = transport;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Command macro = new SequentialCommandGroup(
			new ParallelCommandGroup(
				drivetrain.makeDriveToAmpCommand(),
				new AmpSetup(elevator, pivot)
			),
			new ScoreAmpCommand(shooter, transport).withTimeout(Constants.AmpConstants.allowedShootTime),
			new StowElevatorCommand(elevator, pivot)
		).onlyWhile(RobotContainer.getInstance()::getAmpButton);
    macro.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
