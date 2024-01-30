package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SysIdRoutineBot {
  // The robot's subsystems
//  private Shooter m_SysIDSubsystem = new Shooter();
 private Elevator m_SysIDSubsystem = new Elevator();

  // The driver's controller
  CommandXboxController m_operatorController =
      new CommandXboxController(Constants.InputConstants.kOperatorControllerPort);

  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called during {@link Robot#robotInit()}.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {

    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    m_operatorController.a().whileTrue(m_SysIDSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_operatorController.b().whileTrue(m_SysIDSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_operatorController.x().whileTrue(m_SysIDSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_operatorController.y().whileTrue(m_SysIDSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return m_SysIDSubsystem.run(() -> {});
  }
}