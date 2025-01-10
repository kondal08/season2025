package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class FeederIOSim extends GenericRollerSystemIOSim implements FeederIO {
  public FeederIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
