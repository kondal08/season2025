package frc.robot.subsystems.intakeCoral;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class IntakeCoralIOSim extends GenericRollerSystemIOSim implements IntakeCoralIO {
  public IntakeCoralIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
