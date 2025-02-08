package frc.robot.subsystems.intakeCoral;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class IntakeCoralIOMax extends GenericRollerSystemIOSparkMax implements IntakeCoralIO {
  public IntakeCoralIOMax() {
    super(IntakeCoralConstants.INTAKE_CORAL_ID, 40, false, true, 0.0);
  }
}
