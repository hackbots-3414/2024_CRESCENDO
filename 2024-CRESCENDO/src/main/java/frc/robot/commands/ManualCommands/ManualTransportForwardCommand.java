// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transport;
public class ManualTransportForwardCommand extends Command {
  private Transport transport;
  /** Creates a new ManualTransportForwardCommand. */
  public ManualTransportForwardCommand(Transport transport) {

  // Use addRequirements() here to declare subsystem dependencies.
    this.transport = transport;
    addRequirements(transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transport.setMotor(Constants.TransportConstants.transportEjectSpeed * -1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transport.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
