package frc.robot.subsystems.elevator;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class ElevatorIOReal extends GenericRollerSystemIOSparkMax implements ElevatorIO {
  public ElevatorIOReal(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    super(id, currentLimitAmps, invert, brake, reduction);
  }
}
