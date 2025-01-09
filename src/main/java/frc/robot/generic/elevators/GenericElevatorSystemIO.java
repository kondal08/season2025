package frc.robot.generic.elevators;

import edu.wpi.first.units.measure.Distance;
import frc.robot.generic.rollers.GenericRollerSystemIO;
import org.littletonrobotics.junction.AutoLog;

public interface GenericElevatorSystemIO {

  @AutoLog
  public abstract class GenericElevatorSystemIOInputs {
    public boolean connected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(GenericRollerSystemIO.GenericRollerSystemIOInputs inputs) {}
  /** Run elevator system to a height */
  default void runPosition(Distance height) {}
  /** Stop elevator system */
  default void stop() {}
}
