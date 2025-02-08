package frc.robot.subsystems.intakeAlgae;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class IntakeAlgaeIOMax extends GenericRollerSystemIOSparkMax implements IntakeAlgaeIO {
  public IntakeAlgaeIOMax() {
    super(IntakeAlgaeConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
