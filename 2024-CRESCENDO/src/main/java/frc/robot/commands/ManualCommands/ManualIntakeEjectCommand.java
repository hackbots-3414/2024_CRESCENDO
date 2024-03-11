// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transport;

public class ManualIntakeEjectCommand extends Command {
  private Intake intake;
  private Transport transport;

  /** Creates a new ManualEjectCommand. */
  public ManualIntakeEjectCommand(Intake intake, Transport transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.transport = transport;
    addRequirements(intake, transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMotor(Constants.IntakeConstants.ejectSpeed);
    transport.setMotor(Constants.TransportConstants.transportEjectSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
