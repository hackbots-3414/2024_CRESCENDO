package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;

public class SysIdRoutineBot {
 private Elevator m_SysIDSubsystem = new Elevator();

  CommandXboxController m_operatorController = new CommandXboxController(Constants.InputConstants.kOperatorControllerPort);

  public void configureBindings() {
    m_operatorController.a().whileTrue(m_SysIDSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_operatorController.b().whileTrue(m_SysIDSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_operatorController.x().whileTrue(m_SysIDSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_operatorController.y().whileTrue(m_SysIDSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return m_SysIDSubsystem.run(() -> {});
  }
}