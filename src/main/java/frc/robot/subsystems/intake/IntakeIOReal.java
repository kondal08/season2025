package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.Hardware.REDUCTION;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class IntakeIOReal extends GenericRollerSystemIOSparkMax implements IntakeIO {
  public IntakeIOReal() {
    super(INTAKE_ID, CURRENT_LIMIT, INVERTED, BRAKE_MODE, REDUCTION);
  }
}
