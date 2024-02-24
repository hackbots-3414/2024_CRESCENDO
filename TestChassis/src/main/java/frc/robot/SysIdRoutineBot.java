package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdRoutineBot {

  CommandXboxController m_operatorController = new CommandXboxController(Constants.InputConstants.kOperatorControllerPort);

  public void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}