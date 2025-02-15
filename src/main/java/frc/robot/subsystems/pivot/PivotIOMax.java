package frc.robot.subsystems.pivot;

import frc.robot.generic.arm.GenericArmSystemIOSparkMax;

public class PivotIOMax extends GenericArmSystemIOSparkMax implements PivotIO {

  public PivotIOMax() {
    super(
        new int[] {PivotConstants.PIVOT_ID},
        new boolean[] {PivotConstants.INVERTED},
        40,
        0.0,
        true,
        1.0,
        PivotConstants.kP,
        PivotConstants.kI,
        PivotConstants.kD);
  }
}
