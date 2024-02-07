import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.Intake;

public class IntakeTest implements AutoCloseable {
   static final double DELTA = 1e-3; // acceptable deviation range

   private Intake intake;
   private TalonFXSimState intakeMotorSim;

   @Override
   public void close() {
      /* destroy our TalonFX object */
      try {
         intake.close();
      } catch (Exception e) {
         System.out.println("IntakeTest.java could not close Intake Object");
      }
   }

   @BeforeEach
   public void constructDevices() {
      assert HAL.initialize(500, 0);

      /* create the TalonFX */
      intake = new Intake();
      intakeMotorSim = intake.getSimState();

      /* enable the robot */
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      /* delay ~100ms so the devices can start up and enable */
      Timer.delay(0.100);
   }

   @AfterEach
   void shutdown() {
      /* destroy our TalonFX object */
      try {
         intake.close();
      } catch (Exception e) {
         System.out.println("ElevatorTest.java could not close Elevator Object");
      }
   }

   @Test
   public void robotIsEnabled() {
      /* verify that the robot is enabled */
      assertEquals(DriverStation.isEnabled(), true);
   }

   // @Test
   public void motorDrives() {
      /* set the voltage supplied by the battery */
      intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      var dutyCycle = intake.getMotorDutyCycle();

      /* wait for a fresh duty cycle signal */
      dutyCycle.waitForUpdate(0.100);
      /* verify that the motor output is zero */
      assertEquals(dutyCycle.getValue(), 0.0, DELTA);

      /* request 100% output */
      intake.setControl(new DutyCycleOut(1.0));
      /* wait for the control to apply */
      Timer.delay(0.020);

      /* wait for a new duty cycle signal */
      dutyCycle.waitForUpdate(0.100);
      /* verify that the motor output is 1.0 */
      assertEquals(dutyCycle.getValue(), 1.0, DELTA);
   }
}