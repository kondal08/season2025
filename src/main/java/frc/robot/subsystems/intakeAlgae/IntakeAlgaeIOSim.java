package frc.robot.subsystems.intakeAlgae;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class IntakeAlgaeIOSim extends GenericRollerSystemIOSim implements IntakeAlgaeIO {
  public IntakeAlgaeIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
