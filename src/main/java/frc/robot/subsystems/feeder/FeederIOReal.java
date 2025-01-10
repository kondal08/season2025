package frc.robot.subsystems.feeder;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class FeederIOReal extends GenericRollerSystemIOSparkMax implements FeederIO {
  public FeederIOReal(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    super(id, currentLimitAmps, invert, brake, reduction);
  }
}
