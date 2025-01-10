// change between sim and sparkmax files

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class ElevatorIOSim extends GenericRollerSystemIOSim implements ElevatorIO {
  public ElevatorIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
