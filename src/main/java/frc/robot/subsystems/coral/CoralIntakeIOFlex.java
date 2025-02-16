package frc.robot.subsystems.coral;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkFlex;

public class CoralIntakeIOFlex extends GenericRollerSystemIOSparkFlex implements CoralIntakeIO {
  public CoralIntakeIOFlex() {
    super(CoralIntakeConstants.INTAKE_CORAL_ID, 40, false, true, 0.0);
  }
}
