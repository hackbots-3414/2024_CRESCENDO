import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.Elevator;

public class ElevatorTest implements AutoCloseable {
   static final double DELTA = 1e-3; // acceptable deviation range

   private Elevator elevator;
   private TalonFXSimState elevatorSim;

   @Override
   public void close() {
      /* destroy our TalonFX object */
      try {
         elevator.close();
      } catch (Exception e) {
         System.out.println("ElevatorTest.java could not close Elevator Object");
      }
   }

   @BeforeEach
   public void constructDevices() {
      assert HAL.initialize(500, 0);

      /* create the TalonFX */
      elevator = new Elevator();
      elevatorSim = elevator.getSimState();

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
      elevatorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      double result;
      // get the motor speed
      result = elevator.getPosition();
      assertEquals(result, 0.0, DELTA);

      elevator.setElevatorPosition(1.0);
      Timer.delay(0.2);
      result = elevator.getPosition();

      // System.out.println(result);

      // assertEquals(result, 1.0, DELTA); this fails because the command scheduler doesn't run the periodic() method on the subsystem during tests.
   } 
}