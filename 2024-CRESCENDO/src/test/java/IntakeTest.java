import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
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
      try {
         intake.close();
      } catch (Exception e) {
         System.out.println("IntakeTest.java could not close Intake Object");
      }
   }

   @BeforeEach
   public void constructDevices() {
      assert HAL.initialize(500, 0);

      intake = new Intake();
      intakeMotorSim = intake.getSimState();

      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      Timer.delay(0.100);
   }

   @AfterEach
   void shutdown() {
      try {
         intake.close();
      } catch (Exception e) {
         System.out.println("ElevatorTest.java could not close Elevator Object");
      }
   }

   @Test
   public void motorDrives() {
      intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      var dutyCycle = intake.getMotorDutyCycle();
      dutyCycle.waitForUpdate(0.100);
      assertEquals(dutyCycle.getValue(), 0.0, DELTA);

      intake.setControl(new DutyCycleOut(1.0));
      Timer.delay(0.020);

      dutyCycle.waitForUpdate(0.100);
      assertEquals(dutyCycle.getValue(), 1.0, DELTA);
   }
}