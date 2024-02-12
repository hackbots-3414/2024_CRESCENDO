import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.Transport;

public class TransportTest implements AutoCloseable {
   static final double DELTA = 1e-3; // acceptable deviation range

   private Transport transport;
   
   private TalonFXSimState transportSim;

   @Override
   public void close() {
      /* destroy our TalonFX object */
      try {
         transport.close();
      } catch (Exception e) {
         System.out.println("transportTest.java could not close transport Object");
      }
   }

   @BeforeEach
   public void constructDevices() {
      assert HAL.initialize(500, 0);

      /* create the TalonFX */
      transport = new Transport();
      transportSim = transport.getSimState();

      /* enable the robot */
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      /* delay ~100ms so the devices can start up and enable */
      Timer.delay(0.100);
   }

   @AfterEach
   void shutdown() {
      close();
   }

   // @Test
   public void robotIsEnabled() {
      /* verify that the robot is enabled */
      assertEquals(DriverStation.isEnabled(), true);
   }

   // @Test
   public void motorMoves() {
      /* set the voltage supplied by the battery */
      transportSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      transport.stopMotor();

      StatusSignal<Double> dutyCycle = transport.getMotorDutyCycle();

      transport.setControl(new DutyCycleOut(0.0));
      Timer.delay(0.01);
      dutyCycle.waitForUpdate(0.1);

      assertEquals(dutyCycle.getValue(), 0.0, DELTA);

      transport.setControl(new DutyCycleOut(1.0));
      Timer.delay(0.02);
      dutyCycle.waitForUpdate(0.2);
      assertEquals(dutyCycle.getValue(), 1.0, DELTA);
   }
}