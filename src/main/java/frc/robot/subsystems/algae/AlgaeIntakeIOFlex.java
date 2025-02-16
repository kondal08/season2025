package frc.robot.subsystems.algae;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class AlgaeIntakeIOFlex extends GenericRollerSystemIOSparkMax implements AlgaeIntakeIO {
  public AlgaeIntakeIOFlex() {
    super(AlgaeIntakeConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
