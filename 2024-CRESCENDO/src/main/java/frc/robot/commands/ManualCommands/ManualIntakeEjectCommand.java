package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transport;

public class ManualIntakeEjectCommand extends Command {
  private Intake intake;
  private Transport transport;

  public ManualIntakeEjectCommand(Intake intake, Transport transport) {
    addRequirements(intake, transport);
    this.intake = intake;
    this.transport = transport;
  }

  @Override
  public void initialize() {
    intake.setEject();
    transport.setEject();
  }
  
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
  }
}
