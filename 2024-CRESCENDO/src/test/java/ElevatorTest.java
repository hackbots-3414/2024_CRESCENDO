import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.Elevator;

public class ElevatorTest implements AutoCloseable {
   static final double DELTA = 1e-3;

   private Elevator elevator;
   private TalonFXSimState elevatorSim;

   @Override
   public void close() {
      try {
         elevator.close();
      } catch (Exception e) {
         System.out.println("ElevatorTest.java could not close Elevator Object");
      }
   }

   @BeforeEach
   public void constructDevices() {
      assert HAL.initialize(500, 0);

      elevator = new Elevator();
      elevatorSim = elevator.getSimState();

      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      
      Timer.delay(0.100);
   }

   @AfterEach
   void shutdown() {
      close();
   }

   @Test
   public void motorMoves() {
      elevatorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      double result = elevator.getPosition();
      assertEquals(result, 0.0, DELTA);

      elevator.setElevatorPosition(1.0);
      Timer.delay(0.2);
      result = elevator.getPosition();
   } 
}