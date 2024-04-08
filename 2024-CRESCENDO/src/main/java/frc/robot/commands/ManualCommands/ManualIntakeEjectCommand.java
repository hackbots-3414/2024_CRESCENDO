package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class ManualIntakeEjectCommand extends Command {
  private Intake intake;
  private Transport transport;
  private ShooterPivot pivot;

  public ManualIntakeEjectCommand(Intake intake, Transport transport, ShooterPivot pivot) {
    addRequirements(intake, transport, pivot);
    this.intake = intake;
    this.transport = transport;
    this.pivot = pivot;
  }

  @Override
  public void initialize() {
    pivot.stow();
  }

  @Override
  public void execute() {
      if (pivot.isAtSetpoint()) {
        intake.setEject();
        transport.setEject();
      }
  }
  
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    transport.stopMotor();
  }
}
