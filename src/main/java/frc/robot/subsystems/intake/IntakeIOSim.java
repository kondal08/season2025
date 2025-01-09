package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.Hardware.MOMENT_OF_INERTIA;
import static frc.robot.subsystems.intake.IntakeConstants.Hardware.REDUCTION;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class IntakeIOSim extends GenericRollerSystemIOSim implements IntakeIO {
  public IntakeIOSim() {
    super(DCMotor.getNEO(1), REDUCTION, MOMENT_OF_INERTIA);
  }
}
